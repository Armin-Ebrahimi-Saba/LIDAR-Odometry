"""OPTIONAL advanced extension -- experimental, verify against your bag
before relying on it.

KISS-ICP seeds each ICP step's initial guess with a constant-velocity model
(`last_pose @ last_delta`), i.e. it assumes the platform keeps doing whatever
it did between the previous two scans. During fast turns or jerky motion
this assumption breaks down and ICP can converge to the wrong local minimum.

The Ouster already publishes a fused attitude estimate on `/ouster/imu_att`
at ~100 Hz (per the inventory report: "3D Attitude Quaternions"). This
module replaces only the *rotational* part of the initial guess with the
real attitude change measured between two scan timestamps (via SLERP
interpolation), while keeping the constant-velocity model for translation
(IMU alone gives no position information).

This does not require ROS2, GTSAM, or LIO-SAM -- only the `rosbags` library
and the topic already present in the bag. It is a genuinely pure-Python
"IMU-aided" middle ground between plain KISS-ICP and a full LIO-SAM
integration (see the LIO-SAM section in README.md for the latter).

>>> IMPORTANT <<<
The exact message type / field layout of `/ouster/imu_att` was inferred from
the inventory report's description ("3D Attitude Quaternions"), not from the
actual bag. Before using this module, run:

    python scripts/inspect_bag.py <bag_dir>

and confirm the message type for `/ouster/imu_att`. If it is, e.g.,
`geometry_msgs/msg/QuaternionStamped`, the field is `msg.quaternion`
(adjust `_get_quaternion_field` below accordingly if it differs).
"""
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation, Slerp

from kiss_icp.kiss_icp import KissICP


def _get_quaternion_field(msg):
    """Best-effort extraction of an (x, y, z, w) quaternion from common ROS
    message shapes. Adjust this if `inspect_bag.py` shows a different type
    for /ouster/imu_att in your bag."""
    if hasattr(msg, "orientation"):  # sensor_msgs/Imu
        q = msg.orientation
    elif hasattr(msg, "quaternion"):  # geometry_msgs/QuaternionStamped
        q = msg.quaternion
    elif hasattr(msg, "x") and hasattr(msg, "w"):  # geometry_msgs/Quaternion directly
        q = msg
    else:
        raise AttributeError(
            f"Don't know how to extract a quaternion from message type "
            f"{type(msg)}. Inspect the message fields and update "
            f"_get_quaternion_field()."
        )
    return [q.x, q.y, q.z, q.w]


def load_imu_attitude(bag_dir: str, topic: str = "/ouster/imu_att"):
    """Return (timestamps [s, sorted], rotations [scipy Rotation, sorted])."""
    times, quats = [], []
    typestore = get_typestore(Stores.LATEST)
    with AnyReader([Path(bag_dir)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            available = sorted({c.topic for c in reader.connections})
            raise ValueError(f"Topic '{topic}' not found. Available:\n  " + "\n  ".join(available))
        for connection, t_ns, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            quats.append(_get_quaternion_field(msg))
            times.append(t_ns * 1e-9)

    times = np.asarray(times)
    order = np.argsort(times)
    rotations = Rotation.from_quat(np.asarray(quats)[order])
    return times[order], rotations


def relative_rotation_between(times, rotations, t0: float, t1: float) -> np.ndarray:
    """SLERP-interpolate attitude at t0 and t1 and return the 3x3 rotation
    matrix mapping the platform's orientation at t0 to its orientation at
    t1 (i.e. the incremental rotation between two LiDAR scans)."""
    slerp = Slerp(times, rotations)
    t0c = np.clip(t0, times[0], times[-1])
    t1c = np.clip(t1, times[0], times[-1])
    r0 = slerp([t0c])[0]
    r1 = slerp([t1c])[0]
    return (r1 * r0.inv()).as_matrix()


class IMUAidedKissICP(KissICP):
    """KISS-ICP variant that accepts an externally supplied rotation matrix
    to seed the ICP initial guess, instead of always using the
    constant-velocity assumption. Pass `imu_relative_rotation=None` to fall
    back to standard KISS-ICP behavior for any given frame."""

    def register_frame(self, frame, timestamps, imu_relative_rotation: np.ndarray = None):
        frame = self.preprocessor.preprocess(frame, timestamps, self.last_delta)
        source, frame_downsample = self.voxelize(frame)
        sigma = self.adaptive_threshold.get_threshold()

        if imu_relative_rotation is not None:
            delta = self.last_delta.copy()
            delta[:3, :3] = imu_relative_rotation
            initial_guess = self.last_pose @ delta
        else:
            initial_guess = self.last_pose @ self.last_delta

        new_pose = self.registration.align_points_to_map(
            points=source,
            voxel_map=self.local_map,
            initial_guess=initial_guess,
            max_correspondance_distance=3 * sigma,
            kernel=sigma,
        )

        model_deviation = np.linalg.inv(initial_guess) @ new_pose
        self.adaptive_threshold.update_model_deviation(model_deviation)
        self.local_map.update(frame_downsample, new_pose)
        self.last_delta = np.linalg.inv(self.last_pose) @ new_pose
        self.last_pose = new_pose
        return frame, source
