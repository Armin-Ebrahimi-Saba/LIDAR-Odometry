"""Run the official KISS-ICP package over a sequence of LiDAR scans.

Uses the `kiss-icp` pip package (PRBonn, https://github.com/PRBonn/kiss-icp)
as the odometry engine. Produces, in the world frame seeded by `initial_pose`
(the first GNSS ground-truth point, set by the runner):
  - poses_local.csv : timestamp, x, y, z, qx, qy, qz, qw
  - map_local.pcd   : accumulated 3D point-cloud map

These are georeferenced in the alignment stage (sensys_slam.align).
"""
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from tqdm import tqdm

from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP


def build_kiss_config(cfg: dict) -> KISSConfig:
    """Map the `kiss_icp` block of /configs/config_test1.yaml onto a kiss-icp KISSConfig. The
    block mirrors KISS-ICP's own layout (data / mapping / registration /
    adaptive_threshold); any key left out keeps the package default."""
    kc = cfg.get("kiss_icp", {})
    data = kc.get("data", {})
    mapping = kc.get("mapping", {})
    reg = kc.get("registration", {})
    at = kc.get("adaptive_threshold", {})

    config = KISSConfig()

    config.data.max_range = float(data.get("max_range", config.data.max_range))
    config.data.min_range = float(data.get("min_range", config.data.min_range))
    config.data.deskew = bool(data.get("deskew", config.data.deskew))

    # voxel_size: null in config -> KISS-ICP heuristic (max_range / 100).
    voxel = mapping.get("voxel_size")
    config.mapping.voxel_size = (float(voxel) if voxel
                                 else config.data.max_range / 100.0)
    config.mapping.max_points_per_voxel = int(
        mapping.get("max_points_per_voxel", config.mapping.max_points_per_voxel))

    config.registration.max_num_iterations = int(
        reg.get("max_num_iterations", config.registration.max_num_iterations))
    config.registration.convergence_criterion = float(
        reg.get("convergence_criterion", config.registration.convergence_criterion))
    config.registration.max_num_threads = int(
        reg.get("max_num_threads", config.registration.max_num_threads))

    fixed = at.get("fixed_threshold", config.adaptive_threshold.fixed_threshold)
    config.adaptive_threshold.fixed_threshold = (float(fixed) if fixed is not None
                                                 else None)
    config.adaptive_threshold.initial_threshold = float(
        at.get("initial_threshold", config.adaptive_threshold.initial_threshold))
    config.adaptive_threshold.min_motion_th = float(
        at.get("min_motion_th", config.adaptive_threshold.min_motion_th))

    return config


def run_odometry(dataset, cfg: dict, output_dir: str, initial_pose=None) -> pd.DataFrame:
    """Run KISS-ICP over every scan in `dataset` and write poses + map.

    `dataset` exposes `len()` and `iter_scans()` yielding
    `(timestamp_s, points, point_times)`. `initial_pose` (4x4) seeds the first
    pose -- the world frame anchored at the first ground-truth point.
    """
    import open3d as o3d

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    config = build_kiss_config(cfg)
    engine = KissICP(config)
    # Seed the world frame at the supplied initial pose (first GNSS GT point).
    # KissICP starts at last_pose=I, last_delta=I, so the first frame's ICP
    # initial guess is just this pose (the local map is still empty).
    if initial_pose is not None:
        engine.last_pose = np.asarray(initial_pose, dtype=float)

    n = len(dataset)
    if n == 0:
        raise RuntimeError("Dataset is empty -- nothing to process.")

    # The package's local_map is pruned to max_range around the current pose, so
    # it is not the full trajectory map. Accumulate every frame into our own
    # global map, compacting periodically with a voxel filter to bound memory.
    #
    # `map_voxel_size` is deliberately decoupled from `voxel_size`: the latter
    # drives registration and KISS-ICP's local map (an odometry tuning knob),
    # while this one only sets the resolution of the map deliverable. One point
    # survives per `map_voxel` cell, so it is the hard ceiling on map density.
    mapping_cfg = cfg.get("kiss_icp", {}).get("mapping", {})
    map_voxel = float(mapping_cfg.get("map_voxel_size")
                      or config.mapping.voxel_size)
    global_map = o3d.geometry.PointCloud()
    pending = []

    def _voxel_down(points, voxel):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        return pc.voxel_down_sample(voxel)

    def _compact():
        nonlocal pending
        if not pending:
            return
        global_map.points.extend(
            o3d.utility.Vector3dVector(np.vstack(pending)))
        global_map.points = global_map.voxel_down_sample(map_voxel).points
        pending = []

    records = []
    for timestamp, frame, point_times in tqdm(
            dataset.iter_scans(), total=n, desc="KISS-ICP odometry"):
        # kiss-icp expects per-point timestamps for deskew; pass through (empty
        # array when deskew is off / clouds are already motion-compensated).
        ts = np.asarray(point_times, dtype=float)
        # register_frame returns (deskewed raw scan, registration downsample).
        # Map the raw scan: the registration cloud is voxelized at 1.5x
        # voxel_size, which would cap map density well below map_voxel_size.
        scan, _ = engine.register_frame(frame, ts)
        pose = engine.last_pose
        t = pose[:3, 3]
        q = Rotation.from_matrix(pose[:3, :3]).as_quat()  # [x, y, z, w]
        records.append({
            "timestamp": timestamp,
            "x": t[0], "y": t[1], "z": t[2],
            "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3],
        })

        # scan is in the sensor frame; map it to the world frame for the map.
        # Thin each scan to the map resolution before buffering -- the raw sweep
        # is ~36k points, and anything finer than map_voxel is dropped anyway.
        if scan is not None and len(scan):
            world = (pose[:3, :3] @ np.asarray(scan).T).T + t
            pending.append(np.asarray(_voxel_down(world, map_voxel).points))
        if len(pending) >= 100:
            _compact()

    _compact()

    poses_df = pd.DataFrame.from_records(records)
    poses_path = out / "poses_local.csv"
    poses_df.to_csv(poses_path, index=False)

    map_points = np.asarray(global_map.points)
    map_path = out / "map_local.pcd"
    o3d.io.write_point_cloud(str(map_path), global_map)

    print(f"[odometry] {n} scans processed, {len(map_points)} map points")
    print(f"[odometry] wrote {poses_path}")
    print(f"[odometry] wrote {map_path}")
    return poses_df
