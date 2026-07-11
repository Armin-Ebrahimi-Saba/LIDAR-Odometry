from pathlib import Path
from rosbags.rosbag2 import Reader

with Reader('./rosbag_new') as reader:
    print("Topics in rosbag_new:")
    for topic, conn in reader.topics.items():
        print(f" - {topic}: {conn.msgtype} (Messages: {conn.msgcount})")
    
    print("\nReading first IMU message...")
    for conn, timestamp, rawdata in reader.messages():
        if conn.topic == '/ouster/imu_meas':
            from rosbags.typesys import get_typestore, Stores
            store = get_typestore(Stores.ROS2_HUMBLE)
            msg = store.deserialize_cdr(rawdata, conn.msgtype)
            print("First IMU message Acceleration:")
            print(f" x={msg.linear_acceleration.x}")
            print(f" y={msg.linear_acceleration.y}")
            print(f" z={msg.linear_acceleration.z}")
            print("First IMU message Angular Velocity:")
            print(f" x={msg.angular_velocity.x}")
            print(f" y={msg.angular_velocity.y}")
            print(f" z={msg.angular_velocity.z}")
            break
