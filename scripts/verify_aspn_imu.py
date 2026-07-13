
#!/usr/bin/env python3
"""Sanity-check the reconstructed aspn_msgs/MeasurementIMU against real bag data.
Confirms |accel| is close to 9.81 m/s^2 (gravity) -- validates our .msg field
layout matches what the recording pipeline actually used."""
import sys
import math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from aspn_msgs.msg import MeasurementIMU

def main(bag_path, topic="/ouster/imu_meas", n=5):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id="sqlite3"), ConverterOptions("", ""))
    count = 0
    while reader.has_next():
        t_name, data, t = reader.read_next()
        if t_name == topic:
            msg = deserialize_message(data, MeasurementIMU)
            accel_mag = math.sqrt(sum(a*a for a in msg.meas_accel))
            print(f"accel: {list(msg.meas_accel)}  |accel|={accel_mag:.3f}")
            print(f"gyro:  {list(msg.meas_gyro)}")
            count += 1
            if count >= n:
                break

if __name__ == "__main__":
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "data/Test1_data/rosbag"
    main(bag_path)
