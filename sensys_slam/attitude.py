"""Deskew LiDAR scans with measured PX4 attitude instead of KISS-ICP's
constant-velocity rotational model.

Why this module exists
----------------------
KISS-ICP deskews each sweep by assuming the platform keeps rotating at the
rate implied by the last two scans (constant velocity). When that holds the
built-in deskew is fine; this module replaces only the *rotational* part with
the actual fused attitude measured during the sweep.

Data-source notes (verified against this bag, not assumed)
----------------------------------------------------------
* `/ouster/imu_att` is **all-identity** for the entire recording -- unusable.
* `/fmu/out/vehicle_attitude` (data/px4_msgs/VehicleAttitude, ~100 Hz) carries the
  real fused attitude; its yaw tracks the GNSS course over the run.

The bag embeds no message definitions and px4_msgs is not registered, so the
quaternion is read straight from the CDR payload: a float32[4] at byte
offset 20, stored PX4-style `[w, x, y, z]`. Both facts were reverse-engineered
and validated (unit norm; yaw-vs-course agreement) before use here.

Timing / frame caveats
----------------------
* Attitude and LiDAR are matched on bag-record time. A small constant clock
  offset between the two largely cancels because deskew only uses *relative*
  rotation within a ~100 ms sweep.
* The LiDAR<->FCU extrinsic rotation is unknown, so the measured body-frame
  increment is applied directly in the sensor frame (i.e. extrinsic assumed
  identity). Over a single sweep the rotation is small, so the residual from
  this assumption is second order.
"""
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation, Slerp

PX4_ATTITUDE_TOPIC = "/fmu/out/vehicle_attitude"
_QUAT_BYTE_OFFSET = 20          # float32[4] q within the CDR payload
_WXYZ_TO_XYZW = [1, 2, 3, 0]    # PX4 stores [w,x,y,z]; scipy wants [x,y,z,w]


def _read_attitude(bag_dir: str, topic: str = PX4_ATTITUDE_TOPIC):
    """Read the raw PX4 attitude stream -> (times[s], Rotation) with strictly
    increasing timestamps. Quaternions are body(FRD)->NED (see module docstring)."""
    import struct

    times, quats = [], []
    typestore = get_typestore(Stores.LATEST)
    with AnyReader([Path(bag_dir)], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            available = sorted({c.topic for c in reader.connections})
            raise ValueError(
                f"Attitude topic '{topic}' not found in {bag_dir}.\n"
                f"Available topics:\n  " + "\n  ".join(available)
            )
        for connection, t_ns, rawdata in reader.messages(connections=conns):
            q = struct.unpack_from("<4f", bytes(rawdata), _QUAT_BYTE_OFFSET)
            if 0.98 < float(np.linalg.norm(q)) < 1.02:  # skip any uninitialized samples
                times.append(t_ns * 1e-9)
                quats.append(q)

    if len(times) < 2:
        raise RuntimeError(f"Too few valid attitude samples on '{topic}' ({len(times)}).")

    times = np.asarray(times)
    order = np.argsort(times)
    times = times[order]
    quats = np.asarray(quats)[order][:, _WXYZ_TO_XYZW]
    keep = np.concatenate([[True], np.diff(times) > 0])   # Slerp needs strictly increasing t
    return times[keep], Rotation.from_quat(quats[keep])


def load_attitude_deskewer(bag_dir: str, topic: str = PX4_ATTITUDE_TOPIC):
    """Read the attitude stream and return an `AttitudeDeskewer`."""
    times, rots = _read_attitude(bag_dir, topic)
    return AttitudeDeskewer(times, rots)

class AttitudeDeskewer:
    """Rotates each point of a sweep to the orientation at the sweep end,
    using SLERP-interpolated measured attitude."""

    def __init__(self, times: np.ndarray, rotations: Rotation):
        self.t0 = float(times[0])
        self.t1 = float(times[-1])
        self._slerp = Slerp(times, rotations)

    def deskew(self, points: np.ndarray, point_times_s: np.ndarray, scan_end_time: float) -> np.ndarray:
        """Return motion-compensated points.

        points: (N, 3) sensor-frame points.
        point_times_s: (N,) per-point time within the sweep, seconds (0..~0.1).
        scan_end_time: bag time (s) of the sweep end == deskew reference.
        """
        if len(points) == 0 or point_times_s.size == 0:
            return points

        sweep = float(point_times_s.max())
        # Collapse to the (<=2048) distinct column times for a cheap SLERP.
        uniq, inv = np.unique(point_times_s, return_inverse=True)
        abs_t = np.clip(scan_end_time - (sweep - uniq), self.t0, self.t1)

        rot_pts = self._slerp(abs_t)                                  # (M,) rotations
        rot_ref = self._slerp([np.clip(scan_end_time, self.t0, self.t1)])[0]
        rel = (rot_ref.inv() * rot_pts).as_matrix()                   # (M, 3, 3)
        return np.einsum("mij,mj->mi", rel[inv], points)
