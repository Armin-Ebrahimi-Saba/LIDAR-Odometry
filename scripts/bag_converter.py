#!/usr/bin/env python3
"""Convert given rosbag for GLIM: aspn_msgs/MeasurementIMU -> sensor_msgs/msg/imu. Keeps /ouster/points and /tf_static as it is.
Skips camera topics since it is not needed for the core GLIM pipeline.

Usage: python3 bag_converter.py <input_bag_dir> <output_bag_dir>
"""

import sys
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message, deserialize_message
from aspn_msgs.msg import MeasurementIMU
from sensor_msgs.msg import Imu

KEEP_ASIS = {
    "/ouster/points": "sensor_msgs/msg/PointCloud2",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}
IMU_SRC_TOPIC = "/ouster/imu_meas"
IMU_DST_TOPIC = "/imu/data"


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

    out.orientation_covariance[0] = -1.0  # no orientation estimate available
    return out


def main():
    if len(sys.argv) != 3:
        print("Usage: convert_bag_for_glim.py <input_bag_dir> <output_bag_dir>")
        sys.exit(1)
    in_path, out_path = sys.argv[1], sys.argv[2]

    reader = SequentialReader()
    reader.open(StorageOptions(uri=in_path, storage_id="sqlite3"), ConverterOptions("", ""))

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=out_path, storage_id="sqlite3"), ConverterOptions("", ""))
    writer.create_topic(TopicMetadata(id=0, name="/ouster/points", type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(id=1, name="/tf_static", type="tf2_msgs/msg/TFMessage", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(id=2, name=IMU_DST_TOPIC, type="sensor_msgs/msg/Imu", serialization_format="cdr"))

    count = {"points": 0, "tf_static": 0, "imu": 0}
    first_imu_type_printed = False

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in KEEP_ASIS:
            writer.write(topic, data, t)
            count["points" if topic == "/ouster/points" else "tf_static"] += 1
        elif topic == IMU_SRC_TOPIC:
            msg = deserialize_message(data, MeasurementIMU)
            if not first_imu_type_printed:
                print(f"[info] first IMU msg: imu_type={msg.imu_type}"
                      f"(0=INTEGRATED, 1=SAMPLED per ASPN spec ordering -- "
                      f"expect 1/SAMPLED for raw accel/gyro rates)")
                first_imu_type_printed = True
            imu_msg = convert_imu(msg)
            writer.write(IMU_DST_TOPIC, serialize_message(imu_msg), t)
            count["imu"] += 1
    print(f"Done. Wrote: {count}")


if __name__ == "__main__":

    main()

