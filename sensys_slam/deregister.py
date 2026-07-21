"""Undo the world-frame registration of the Ouster clouds back to sensor frame.

The bag's `/ouster/points` clouds are already registered into a fixed world
frame (consecutive scans are ~98% identical even as the platform moves), so
scan-to-scan/scan-to-map odometry sees no ego-motion. To recover sensor-frame
sweeps that *do* carry motion, we undo the registration with the platform's
per-frame pose from PX4 `/fmu/out/vehicle_odometry` (NED, ~100 Hz):

    p_sensor(i) = R(q_i)^T @ (p_world - pos_i)

i.e. `T_px4(i)^-1` applied to each cloud. The result is a body-frame sweep at
frame i (up to a constant, motion-irrelevant LiDAR<->body lever arm, which the
alignment stage absorbs). Running the from-scratch KISS-ICP on these recovers
relative motion again.

NOTE: this re-introduces PX4 motion into the clouds, so the resulting "LiDAR
odometry" is seeded by PX4 (the only motion source available, since the LiDAR's
own ego-motion was baked out by the registration). It demonstrates the engine on
real-motion data and produces a coherent map/trajectory, but is not independent
of PX4.
"""
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from scipy.spatial.transform import Rotation, Slerp

PX4_ODOM_TOPIC = "/fmu/out/vehicle_odometry"
POSE_FRAME_NED = 1

# data/px4_msgs/msg/VehicleOdometry -- not embedded in the bag, registered here.
_VEHICLE_ODOMETRY_MSG = """
uint64 timestamp
uint64 timestamp_sample
uint8 pose_frame
float32[3] position
float32[4] q
uint8 velocity_frame
float32[3] velocity
float32[3] angular_velocity
float32[3] position_variance
float32[3] orientation_variance
float32[3] velocity_variance
uint8 reset_counter
int8 quality
"""


def load_deregisterer(bag_dir: str, topic: str = PX4_ODOM_TOPIC):
    """Read PX4 odometry and return a `Deregisterer`."""
    ts = get_typestore(Stores.LATEST)
    ts.register(get_types_from_msg(_VEHICLE_ODOMETRY_MSG, "data/px4_msgs/msg/VehicleOdometry"))

    times, pos, quat = [], [], []
    with AnyReader([Path(bag_dir)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            available = sorted({c.topic for c in reader.connections})
            raise ValueError(
                f"Odometry topic '{topic}' not found in {bag_dir}.\n"
                f"Available topics:\n  " + "\n  ".join(available))
        for conn, t_ns, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            if m.pose_frame != POSE_FRAME_NED:
                continue
            p = np.asarray(m.position, float)
            q = np.asarray(m.q, float)  # [w, x, y, z]
            if np.all(np.isfinite(p)) and 0.9 < np.linalg.norm(q) < 1.1:
                times.append(t_ns * 1e-9)
                pos.append(p)
                quat.append(q[[1, 2, 3, 0]])  # -> [x, y, z, w]

    if len(times) < 2:
        raise RuntimeError(f"Too few valid PX4 odometry samples on '{topic}' ({len(times)}).")

    times = np.asarray(times)
    order = np.argsort(times)
    times, pos, quat = times[order], np.asarray(pos)[order], np.asarray(quat)[order]
    keep = np.concatenate([[True], np.diff(times) > 0])  # Slerp needs strictly increasing
    return Deregisterer(times[keep], pos[keep], Rotation.from_quat(quat[keep]))


def load_gnss_deregisterer(cfg: dict):
    """Build a translation-only de-registerer from the GNSS ground-truth track.

    The clouds' world frame is closest to (but not exactly) the GNSS ENU frame,
    so subtracting the GNSS ENU position at each frame's time restores most of
    the ego-motion. Orientation is left as-is (the world frame is
    orientation-consistent), which an empirical recipe test showed is adequate
    and far better than the PX4 pose. NOTE: this uses the evaluation ground
    truth to build the input, so the resulting trajectory is *circular* with the
    GNSS reference -- it is a demonstration, not an independent evaluation.
    """
    from .groundtruth import load_ground_truth_for_run
    from .geo import geodetic_to_enu
    gt = load_ground_truth_for_run(cfg)
    lat0, lon0, alt0 = float(gt.lat.iloc[0]), float(gt.lon.iloc[0]), float(gt.alt.iloc[0])
    enu = geodetic_to_enu(gt.lat.values, gt.lon.values, gt.alt.values, lat0, lon0, alt0)
    return GnssDeregisterer(gt.timestamp.values.astype(float), enu)


class GnssDeregisterer:
    """Translation-only de-registration using the GNSS ENU track."""

    def __init__(self, times: np.ndarray, enu: np.ndarray):
        self.t0, self.t1 = float(times[0]), float(times[-1])
        self._t = times
        self._enu = enu

    def deregister(self, points: np.ndarray, t: float) -> np.ndarray:
        if len(points) == 0:
            return points
        tc = float(np.clip(t, self.t0, self.t1))
        p = np.array([np.interp(tc, self._t, self._enu[:, k]) for k in range(3)])
        return points - p


# ENU <- NED axis map: E=N_ned, N=E_ned, U=-D_ned.
_NED2ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def load_gnss_attitude_deregisterer(cfg: dict, attitude_topic: str = "/fmu/out/vehicle_attitude"):
    """De-register with GNSS ENU position (translation) AND PX4 attitude
    (rotation, body->NED->ENU). Accounting for the vehicle's rotation should
    reduce the heading drift/spiral that translation-only de-registration
    produces through turns. Still circular with the GNSS ground truth."""
    from .groundtruth import load_ground_truth_for_run
    from .geo import geodetic_to_enu
    from .attitude import load_attitude_deskewer
    gt = load_ground_truth_for_run(cfg)
    lat0, lon0, alt0 = float(gt.lat.iloc[0]), float(gt.lon.iloc[0]), float(gt.alt.iloc[0])
    enu = geodetic_to_enu(gt.lat.values, gt.lon.values, gt.alt.values, lat0, lon0, alt0)
    att = load_attitude_deskewer(cfg["paths"]["bag_dir"], attitude_topic)
    return GnssAttitudeDeregisterer(gt.timestamp.values.astype(float), enu, att)


class GnssAttitudeDeregisterer:
    """GNSS ENU position + PX4 attitude de-registration."""

    def __init__(self, times: np.ndarray, enu: np.ndarray, att):
        self.t0, self.t1 = float(times[0]), float(times[-1])
        self._t = times
        self._enu = enu
        self._att = att  # attitude.AttitudeDeskewer (carries a Slerp, body->NED)

    def deregister(self, points: np.ndarray, t: float) -> np.ndarray:
        if len(points) == 0:
            return points
        tc = float(np.clip(t, self.t0, self.t1))
        p = np.array([np.interp(tc, self._t, self._enu[:, k]) for k in range(3)])
        ta = float(np.clip(t, self._att.t0, self._att.t1))
        R = _NED2ENU @ self._att._slerp([ta])[0].as_matrix()  # body -> ENU
        return (points - p) @ R  # R^T @ (points - p)


class Deregisterer:
    """Maps a world-registered cloud back to the body/sensor frame at time t."""

    def __init__(self, times: np.ndarray, positions: np.ndarray, rotations: Rotation):
        self.t0, self.t1 = float(times[0]), float(times[-1])
        self._times = times
        self._pos = positions
        self._slerp = Slerp(times, rotations)

    def pose_at(self, t: float):
        t = float(np.clip(t, self.t0, self.t1))
        R = self._slerp([t])[0].as_matrix()
        p = np.array([np.interp(t, self._times, self._pos[:, k]) for k in range(3)])
        return R, p

    def deregister(self, points: np.ndarray, t: float) -> np.ndarray:
        if len(points) == 0:
            return points
        R, p = self.pose_at(t)
        return (points - p) @ R  # R^T @ (points - p), vectorized
