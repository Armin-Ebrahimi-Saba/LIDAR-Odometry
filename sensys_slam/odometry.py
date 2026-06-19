"""Run KISS-ICP odometry/SLAM over a sequence of .laz scans.

Produces, in the local SLAM frame (arbitrary origin = first scan pose):
  - poses_local.csv  : timestamp, x, y, z, qx, qy, qz, qw
  - map_local.pcd     : accumulated 3D point cloud map

These are georeferenced in the alignment stage (sensys_slam.align), which is
where the local SLAM frame gets tied to the GNSS ground truth.
"""
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from tqdm import tqdm

from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP

from .lidar_io import LazScanDataset


def build_kiss_config(cfg: dict) -> KISSConfig:
    kc = cfg.get("kiss_icp", {})
    config = KISSConfig()
    config.data.max_range = kc.get("max_range", 100.0)
    config.data.min_range = kc.get("min_range", 0.0)
    config.data.deskew = kc.get("deskew", False)
    voxel_size = kc.get("voxel_size")
    config.mapping.voxel_size = voxel_size if voxel_size else config.data.max_range / 100.0
    return config


def run_odometry(dataset: LazScanDataset, cfg: dict, output_dir: str) -> pd.DataFrame:
    """Run KISS-ICP over every scan in `dataset` and write poses + map to disk."""
    import open3d as o3d

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    odometry = KissICP(config=build_kiss_config(cfg))

    n = len(dataset)
    if n == 0:
        raise RuntimeError("Dataset is empty -- nothing to process.")

    records = []
    for idx in tqdm(range(n), desc="KISS-ICP odometry"):
        frame, point_times = dataset[idx]
        odometry.register_frame(frame, point_times)
        pose = odometry.last_pose  # 4x4 homogeneous transform
        t = pose[:3, 3]
        q = Rotation.from_matrix(pose[:3, :3]).as_quat()  # [x, y, z, w]
        records.append(
            {
                "timestamp": dataset.scan_timestamp(idx),
                "x": t[0], "y": t[1], "z": t[2],
                "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3],
            }
        )

    poses_df = pd.DataFrame.from_records(records)
    poses_path = output_dir / "poses_local.csv"
    poses_df.to_csv(poses_path, index=False)

    map_points = odometry.local_map.point_cloud()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(map_points)
    map_path = output_dir / "map_local.pcd"
    o3d.io.write_point_cloud(str(map_path), pcd)

    print(f"[odometry] {n} scans processed, {len(map_points)} map points")
    print(f"[odometry] wrote {poses_path}")
    print(f"[odometry] wrote {map_path}")
    return poses_df
