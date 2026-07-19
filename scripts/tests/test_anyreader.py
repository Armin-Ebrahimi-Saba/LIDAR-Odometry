from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg
from rosbags.typesys.store import Typestore

aspn_type_header_msg = """
uint32 vendor_id
uint64 device_id
uint32 context_id
uint16 sequence_id
"""
aspn_type_timestamp_msg = """
int64 elapsed_nsec
"""
aspn_type_integrity_msg = """
int32 integrity_method
bool integrity_value_valid
float64 integrity_value
"""
aspn_measurement_imu_msg = """
aspn_msgs/TypeHeader header
aspn_msgs/TypeTimestamp time_of_validity
int32 imu_type
float64[3] meas_accel
float64[3] meas_gyro
uint8 num_integrity
aspn_msgs/TypeIntegrity[] integrity
"""

store = Typestore()
store.register(get_types_from_msg(aspn_type_header_msg, 'aspn_msgs/msg/TypeHeader'))
store.register(get_types_from_msg(aspn_type_timestamp_msg, 'aspn_msgs/msg/TypeTimestamp'))
store.register(get_types_from_msg(aspn_type_integrity_msg, 'aspn_msgs/msg/TypeIntegrity'))
store.register(get_types_from_msg(aspn_measurement_imu_msg, 'aspn_msgs/msg/MeasurementIMU'))

with AnyReader([Path('./rosbag')], default_typestore=store) as reader:
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/ouster/imu_meas':
            msg = reader.deserialize(rawdata, connection.msgtype)
            print("Successfully deserialized IMU:")
            print(msg.meas_accel)
            print(msg.meas_gyro)
            break
