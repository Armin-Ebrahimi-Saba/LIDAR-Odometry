"""PX4 flight-controller odometry as the local trajectory source.

WHY THIS EXISTS
---------------
The exported ``.laz`` point clouds in this dataset are NOT raw sensor-frame
scans -- they are already registered into a single fixed world frame (an
upstream mapping system produced them). Two scans 24 m apart in real motion
share ~94 % of their points at identical coordinates (median nearest-neighbour
0.01 m). Frame-to-frame LiDAR odometry such as KISS-ICP recovers motion from
how the scene *shifts* between scans; here the scene does not shift, so KISS-ICP
sees ~no motion, its trajectory collapses, and (worse) it can diverge to NaN.

The platform, however, logs its own fused state estimate on
``/fmu/out/vehicle_odometry`` (PX4, ~100 Hz, NED). Its trajectory matches the
GNSS ground truth in extent and path length (≈585 m vs ≈591 m), so we use it as
the local trajectory and keep the world-frame ``.laz`` clouds purely as the 3D
map deliverable. Everything downstream (align -> velocity -> evaluate) is
unchanged: it only needs a ``poses_local.csv`` that actually encodes motion.

The message definition for ``px4_msgs/msg/VehicleOdometry`` is not embedded in
the bag, so we register it here from its known field layout.
"""
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# px4_msgs/msg/VehicleOdometry -- all-primitive fields, no nested custom types.
# pose_frame: 1 = NED, 2 = FRD.  position = [x, y, z], q = [w, x, y, z].
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

POSE_FRAME_NED = 1


def _typestore_with_px4():
    ts = get_typestore(Stores.LATEST)
    ts.register(get_types_from_msg(_VEHICLE_ODOMETRY_MSG, "px4_msgs/msg/VehicleOdometry"))
    return ts


def run_px4_odometry(cfg: dict, output_dir: str) -> pd.DataFrame:
    """Read PX4 ``/fmu/out/vehicle_odometry`` from the bag, crop to the run
    time window, thin to ~10 Hz, and write ``poses_local.csv`` in the same
    schema KISS-ICP would have produced (timestamp, x, y, z, qx, qy, qz, qw).

    The local frame is PX4 NED (x=North, y=East, z=Down); the alignment stage
    rotates it onto the GNSS ENU frame, so no manual axis swap is needed here.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    bag_dir = cfg["paths"]["bag_dir"]
    odom_cfg = cfg.get("odometry", {})
    topic = odom_cfg.get("px4_topic", "/fmu/out/vehicle_odometry")
    target_hz = float(odom_cfg.get("px4_target_hz", 10.0))
    t_start = float(cfg["run"]["start_time"])
    t_end = float(cfg["run"]["end_time"])

    rows = []
    ts = _typestore_with_px4()
    with AnyReader([Path(bag_dir)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            available = sorted({c.topic for c in reader.connections})
            raise ValueError(
                f"Topic '{topic}' not found in bag at {bag_dir}.\n"
                f"Available topics:\n  " + "\n  ".join(available)
            )
        for conn, _t_ns, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            t = msg.timestamp * 1e-6  # PX4 timestamp is microseconds -> Unix seconds
            if t < t_start or t > t_end:
                continue
            if msg.pose_frame != POSE_FRAME_NED:
                raise RuntimeError(
                    f"Expected NED pose_frame ({POSE_FRAME_NED}) on {topic}, "
                    f"got {msg.pose_frame}. Position axis handling assumes NED."
                )
            px, py, pz = msg.position
            qw, qx, qy, qz = msg.q
            if not np.isfinite([px, py, pz]).all():
                continue
            rows.append((t, float(px), float(py), float(pz),
                         float(qx), float(qy), float(qz), float(qw)))

    if not rows:
        raise RuntimeError(
            f"No '{topic}' messages fell inside the run window "
            f"[{t_start}, {t_end}]. Check run.start_time/end_time in config.yaml."
        )

    df = pd.DataFrame(rows, columns=["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
    df = df.sort_values("timestamp").reset_index(drop=True)
    df = _thin_to_rate(df, target_hz)

    poses_path = output_dir / "poses_local.csv"
    df.to_csv(poses_path, index=False)
    dur = df["timestamp"].max() - df["timestamp"].min()
    plen = np.sqrt(np.diff(df.x) ** 2 + np.diff(df.y) ** 2).sum()
    print(f"[px4_odometry] {len(df)} poses over {dur:.1f}s "
          f"(~{len(df)/dur:.0f} Hz), 2D path length {plen:.1f} m")
    print(f"[px4_odometry] wrote {poses_path}")
    return df


def _thin_to_rate(df: pd.DataFrame, target_hz: float) -> pd.DataFrame:
    """Keep roughly `target_hz` evenly-time-spaced rows (PX4 logs at ~100 Hz;
    ~10 Hz keeps the trajectory faithful while matching the velocity-stage
    smoothing window tuned for the original 10 Hz LiDAR cadence)."""
    if target_hz <= 0 or len(df) < 3:
        return df
    t = df["timestamp"].values
    keep = [0]
    min_dt = 1.0 / target_hz
    last = t[0]
    for i in range(1, len(t)):
        if t[i] - last >= min_dt:
            keep.append(i)
            last = t[i]
    return df.iloc[keep].reset_index(drop=True)


def build_map_from_world_frame_laz(manifest_df: pd.DataFrame, output_dir: str,
                                   stride: int = 50, voxel_size: float = 0.5) -> None:
    """Build ``map_local.pcd`` by merging the world-frame ``.laz`` clouds.

    These clouds are already in a single consistent frame, so the 3D map is
    just a voxel-downsampled union of a strided subset of scans (using every
    scan would be redundant and huge given their ~99 % inter-scan overlap).
    """
    import open3d as o3d

    output_dir = Path(output_dir)
    merged = o3d.geometry.PointCloud()
    rows = manifest_df.iloc[::stride]
    for fp in rows["filepath"]:
        pcd = o3d.io.read_point_cloud(str(fp))
        merged += pcd
        if voxel_size and voxel_size > 0:
            merged = merged.voxel_down_sample(voxel_size)

    map_path = output_dir / "map_local.pcd"
    o3d.io.write_point_cloud(str(map_path), merged)
    print(f"[px4_odometry] merged {len(rows)} world-frame scans -> "
          f"{len(merged.points)} map points")
    print(f"[px4_odometry] wrote {map_path}")
