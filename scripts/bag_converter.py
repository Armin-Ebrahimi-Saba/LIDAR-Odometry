#!/usr/bin/env python3
"""Convert rosbag for GLIM: aspn_msgs/MeasurementIMU -> sensor_msgs/msg/Imu.

Applies a valid SO(3) rotation (180 deg around X-axis) to both accel and gyro
to maintain a valid right-handed coordinate frame.
Renames /ouster/points' 'timeoffset' field to 'time', dynamically handling
uint32 vs float32 dtypes before scaling from ms to s.
"""

import sys
import numpy as np
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message, deserialize_message
from aspn_msgs.msg import MeasurementIMU
from sensor_msgs.msg import Imu, PointCloud2, PointField

IMU_SRC_TOPIC = "/ouster/imu_meas"
IMU_DST_TOPIC = "/imu/data"
POINTS_TOPIC = "/ouster/points"
TIME_FIELD_SRC = "timeoffset"
TIME_FIELD_DST = "time"

# ROS PointField Datatypes: 5=INT32, 6=UINT32, 7=FLOAT32, 8=FLOAT64
PF_DTYPE_MAP = {
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64
}

def convert_imu(msg: MeasurementIMU, apply_180_x_rot: bool = True) -> Imu:
    out = Imu()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = msg.header.frame_id if msg.header.frame_id else "os_imu"

    ax, ay, az = msg.meas_accel[0], msg.meas_accel[1], msg.meas_accel[2]
    gx, gy, gz = msg.meas_gyro[0], msg.meas_gyro[1], msg.meas_gyro[2]

    if apply_180_x_rot:
        # Valid SO(3) rotation (Rx = pi): [x, y, z] -> [x, -y, -z]
        # Applied to BOTH accelerometer and gyroscope
        out.linear_acceleration.x = float(ax)
        out.linear_acceleration.y = float(-ay)
        out.linear_acceleration.z = float(-az)

        out.angular_velocity.x = float(gx)
        out.angular_velocity.y = float(-gy)
        out.angular_velocity.z = float(-gz)
    else:
        # Pass raw right-handed readings directly
        out.linear_acceleration.x = float(ax)
        out.linear_acceleration.y = float(ay)
        out.linear_acceleration.z = float(az)

        out.angular_velocity.x = float(gx)
        out.angular_velocity.y = float(gy)
        out.angular_velocity.z = float(gz)

    out.orientation_covariance[0] = -1.0
    return out


def fix_points_time_field(raw_data: bytes) -> bytes:
    msg = deserialize_message(raw_data, PointCloud2)

    time_field = None
    for f in msg.fields:
        if f.name == TIME_FIELD_SRC or f.name == TIME_FIELD_DST:
            time_field = f
            break

    if time_field is None:
        return raw_data

    src_dtype = PF_DTYPE_MAP.get(time_field.datatype, np.float32)
    n = msg.width * msg.height
    data = np.frombuffer(bytearray(msg.data), dtype=np.uint8)

    # Read existing timestamps according to their native datatype
    time_view = np.ndarray((n,), dtype=src_dtype, buffer=data, offset=time_field.offset, strides=(msg.point_step,))

    # If it's stored as uint32 or float32 in ms, scale to float32 seconds
    if src_dtype != np.float32:
        # Convert values to float32 seconds
        scaled_times = (time_view.astype(np.float32)) / 1000.0
        # Overwrite buffer with float32 array
        time_view_float = np.ndarray((n,), dtype=np.float32, buffer=data, offset=time_field.offset, strides=(msg.point_step,))
        time_view_float[:] = scaled_times
        time_field.datatype = PointField.FLOAT32
    else:
        # In-place scaling if already float32
        time_view /= 1000.0

    time_field.name = TIME_FIELD_DST
    msg.data = data.tobytes()

    return serialize_message(msg)


def main():
    if len(sys.argv) != 3:
        print("Usage: bag_converter.py <input_bag_dir> <output_bag_dir>")
        sys.exit(1)
    in_path, out_path = sys.argv[1], sys.argv[2]

    reader = SequentialReader()
    reader.open(StorageOptions(uri=in_path, storage_id="sqlite3"), ConverterOptions("", ""))

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=out_path, storage_id="sqlite3"), ConverterOptions("", ""))
    writer.create_topic(TopicMetadata(id=0, name=POINTS_TOPIC, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(id=1, name="/tf_static", type="tf2_msgs/msg/TFMessage", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(id=2, name=IMU_DST_TOPIC, type="sensor_msgs/msg/Imu", serialization_format="cdr"))

    count = {"points": 0, "tf_static": 0, "imu": 0}

    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == POINTS_TOPIC:
            fixed_data = fix_points_time_field(data)
            writer.write(POINTS_TOPIC, fixed_data, t)
            count["points"] += 1

        elif topic == "/tf_static":
            writer.write(topic, data, t)
            count["tf_static"] += 1

        elif topic == IMU_SRC_TOPIC:
            msg = deserialize_message(data, MeasurementIMU)
            imu_msg = convert_imu(msg, apply_180_x_rot=True)
            writer.write(IMU_DST_TOPIC, serialize_message(imu_msg), t)
            count["imu"] += 1

    print(f"Done. Processed messages: {count}")


if __name__ == "__main__":
    main()
