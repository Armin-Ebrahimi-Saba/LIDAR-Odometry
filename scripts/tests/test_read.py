from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import get_types_from_msg, register_types

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
aspn_msgs/msg/TypeHeader header
aspn_msgs/msg/TypeTimestamp time_of_validity
int32 imu_type
float64[3] meas_accel
float64[3] meas_gyro
uint8 num_integrity
aspn_msgs/msg/TypeIntegrity[] integrity
"""

types = {}
types.update(get_types_from_msg(aspn_type_header_msg, 'aspn_msgs/msg/TypeHeader'))
types.update(get_types_from_msg(aspn_type_timestamp_msg, 'aspn_msgs/msg/TypeTimestamp'))
types.update(get_types_from_msg(aspn_type_integrity_msg, 'aspn_msgs/msg/TypeIntegrity'))
types.update(get_types_from_msg(aspn_measurement_imu_msg, 'aspn_msgs/msg/MeasurementIMU'))
register_types(types)

from rosbags.typesys.types import aspn_msgs__msg__MeasurementIMU as MeasurementIMU

with Reader('./rosbag') as reader:
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/ouster/imu_meas':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            print("Successfully deserialized IMU:")
            print(msg.meas_accel)
            print(msg.meas_gyro)
            break
