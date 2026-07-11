import struct
import numpy as np
from pathlib import Path
from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import get_typestore, Stores

store = get_typestore(Stores.ROS2_HUMBLE)
Imu = store.types['sensor_msgs/msg/Imu']
Header = store.types['std_msgs/msg/Header']
Time = store.types['builtin_interfaces/msg/Time']
Vector3 = store.types['geometry_msgs/msg/Vector3']
Quaternion = store.types['geometry_msgs/msg/Quaternion']

inbag_path = Path('./rosbag')
outbag_path = Path('./rosbag_new')

if outbag_path.exists():
    import shutil
    shutil.rmtree(outbag_path)

with Reader(inbag_path) as reader, Writer(outbag_path, version=8) as writer:
    conn_map = {}
    
    # Setup connections for writer
    for conn in reader.connections:
        if conn.topic == '/ouster/points':
            conn_map[conn.id] = writer.add_connection('/ouster/points', conn.msgtype, typestore=store)
        elif conn.topic == '/ouster/imu_meas':
            conn_map[conn.id] = writer.add_connection(
                '/ouster/imu_meas',
                'sensor_msgs/msg/Imu',
                typestore=store
            )

    count = 0
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/ouster/points':
            writer.write(conn_map[connection.id], timestamp, rawdata)
            count += 1
        elif connection.topic == '/ouster/imu_meas':
            payload = rawdata[4:]
            
            # Decode Header
            sec, nanosec, frame_id_len = struct.unpack('<II I', payload[0:12])
            frame_id = payload[12:12+frame_id_len-1].decode('ascii')
            
            # Find offset to float array
            offset = 12 + frame_id_len
            offset = (offset + 7) & ~7 # time_of_validity
            offset += 8 
            offset += 4 # vendor_id
            offset = (offset + 7) & ~7 # device_id
            offset += 8
            offset += 4 # context_id
            offset += 2 # sequence_id
            offset = (offset + 3) & ~3 # imu_type
            offset += 4
            offset = (offset + 7) & ~7 # floats
            
            accel = struct.unpack('<ddd', payload[offset:offset+24])
            gyro = struct.unpack('<ddd', payload[offset+24:offset+48])
            
            msg = Imu(
                header=Header(
                    stamp=Time(sec=sec, nanosec=nanosec),
                    frame_id=frame_id
                ),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                orientation_covariance=np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                angular_velocity=Vector3(x=gyro[0], y=gyro[1], z=gyro[2]),
                angular_velocity_covariance=np.array([0.0]*9),
                linear_acceleration=Vector3(x=accel[0], y=accel[1], z=accel[2]),
                linear_acceleration_covariance=np.array([0.0]*9)
            )
            
            out_rawdata = store.serialize_cdr(msg, 'sensor_msgs/msg/Imu')
            writer.write(conn_map[connection.id], timestamp, out_rawdata)
            count += 1

print(f"Conversion finished. Processed {count} messages.")
