from rosbags.typesys.store import Typestore
import numpy as np

store = Typestore()
Imu = store.types['sensor_msgs/msg/Imu']
Header = store.types['std_msgs/msg/Header']
Time = store.types['builtin_interfaces/msg/Time']

msg = Imu(
    header=Header(
        stamp=Time(sec=123, nanosec=456),
        frame_id='ouster_imu'
    ),
    orientation=np.array([0.0, 0.0, 0.0, 0.0]),
    orientation_covariance=np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    angular_velocity=np.array([1.0, 2.0, 3.0]),
    angular_velocity_covariance=np.array([0.0]*9),
    linear_acceleration=np.array([4.0, 5.0, 6.0]),
    linear_acceleration_covariance=np.array([0.0]*9)
)

raw = store.serialize_cdr(msg, 'sensor_msgs/msg/Imu')
print("Successfully serialized! Length:", len(raw))
