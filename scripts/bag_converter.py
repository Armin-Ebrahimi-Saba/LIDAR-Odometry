#!/usr/bin/env python3
"""Convert given rosbag for GLIM: aspn_msgs/MeasurementIMU -> sensor_msgs/msg/Imu.
Also renames /ouster/points' 'timeoffset' field to 'time' and rescales it from
ms to s, so GLIM's per-point deskewing can use it (confirmed by course staff,
July 2026: timeoffset = (timestamp - first_timestamp) / 1e6, in milliseconds,
relative to the first packet of each scan).
Keeps /tf_static as-is. Skips camera topics (not needed for core GLIM pipeline).

Usage: python3 bag_converter.py <input_bag_dir> <output_bag_dir>
"""

import sys
import numpy as np
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message, deserialize_message
from aspn_msgs.msg import MeasurementIMU
from sensor_msgs.msg import Imu, PointCloud2

IMU_SRC_TOPIC = "/ouster/imu_meas"
IMU_DST_TOPIC = "/imu/data"
POINTS_TOPIC = "/ouster/points"
TIME_FIELD_SRC = "timeoffset"
TIME_FIELD_DST = "time"


def convert_imu(msg: MeasurementIMU) -> Imu:
    out = Imu()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = msg.header.frame_id if msg.header.frame_id else "os_imu"
    out.linear_acceleration.x = -msg.meas_accel[0]
    out.linear_acceleration.y = -msg.meas_accel[1]
    out.linear_acceleration.z = -msg.meas_accel[2]
    out.angular_velocity.x = msg.meas_gyro[0]
    out.angular_velocity.y = msg.meas_gyro[1]
    out.angular_velocity.z = msg.meas_gyro[2]
    out.orientation_covariance[0] = -1.0
    return out


def fix_points_time_field(raw_data: bytes) -> bytes:
    msg = deserialize_message(raw_data, PointCloud2)

    time_field = None
    for f in msg.fields:
        if f.name == TIME_FIELD_SRC:
            time_field = f
            break
    if time_field is None:
        # Field not found (shouldn't happen) -- pass through unchanged
        return raw_data

    n = msg.width * msg.height
    data = np.frombuffer(bytearray(msg.data), dtype=np.uint8)
    time_vals = np.ndarray((n,), dtype=np.float32, buffer=data, offset=time_field.offset, strides=(msg.point_step,))
    time_vals /= 1000.0  # ms -> s

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
    first_imu_type_printed = False

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
            if not first_imu_type_printed:
                print(f"[info] first IMU msg: imu_type={msg.imu_type}")
                first_imu_type_printed = True
            imu_msg = convert_imu(msg)
            writer.write(IMU_DST_TOPIC, serialize_message(imu_msg), t)
            count["imu"] += 1

    print(f"Done. Wrote: {count}")


if __name__ == "__main__":
    main()
