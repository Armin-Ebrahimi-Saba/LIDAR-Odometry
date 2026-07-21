#!/usr/bin/env python3
"""Rebuild the map from a POSE-INDEX subrange of an existing `poses_local.csv`.

Like `rebuild_map.py`, this replays the bag against poses already on disk -- no
ICP -- but it accumulates only a slice of the run, so you can look at one leg in
isolation.

Two differences from `rebuild_map.py` that matter:

  * ranges here are **pose indices**, not bag frame indices. The odometry run may
    have started partway into the bag (`--frames`), so pose i is bag frame
    i + offset; the offset is recovered from `scan_manifest.csv`. `rebuild_map`
    assumes offset 0, which is wrong for any run that did not start at frame 0.
  * IMPORTANT -- the poses are NOT recomputed. Every pose still carries the drift
    the frontend had accumulated by that point in the *full* run. Slicing the map
    is not the same as running the odometry on half the data; it crops what is
    drawn, not what was estimated.

    python scripts/partial_map.py --poses 0 3361 --suffix first_half
"""
import argparse
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import yaml
from tqdm import tqdm

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sensys_slam.frames import build_lidar_to_body        # noqa: E402
from sensys_slam.lidar_io import BagScanDataset, LazScanDataset  # noqa: E402
from rebuild_map import _pose_matrices, _deskew_with_delta       # noqa: E402
from colorize_map import colorize_by_height                      # noqa: E402


def build(config_path, pose_start, pose_end, suffix, map_voxel_size=None,
          height=True):
    import open3d as o3d

    cfg = yaml.safe_load(Path(config_path).read_text())
    out_dir = Path(cfg["paths"]["output_dir"])
    poses = pd.read_csv(out_dir / "poses_local.csv")
    pose_end = min(pose_end, len(poses) - 1)
    T = _pose_matrices(poses)

    mapping = cfg.get("kiss_icp", {}).get("mapping", {})
    voxel = float(map_voxel_size or mapping.get("map_voxel_size")
                  or mapping.get("voxel_size") or 1.0)
    data = cfg.get("kiss_icp", {}).get("data", {})
    min_r, max_r = float(data.get("min_range", 0.0)), float(data.get("max_range", 100.0))

    lidar = cfg.get("lidar", {})
    imu_deskew = bool(lidar.get("imu_deskew", False))
    deskewer = None
    if imu_deskew:
        from sensys_slam.attitude import load_attitude_deskewer, PX4_ATTITUDE_TOPIC
        deskewer = load_attitude_deskewer(
            cfg["paths"]["bag_dir"], lidar.get("attitude_topic", PX4_ATTITUDE_TOPIC))

    manifest = pd.read_csv(out_dir / "scan_manifest.csv")
    offset = int(np.argmin(np.abs(manifest["timestamp"].values
                                  - poses["timestamp"].iloc[0])))
    if abs(manifest["timestamp"].values[offset] - poses["timestamp"].iloc[0]) > 1e-3:
        raise SystemExit("cannot locate pose[0] in scan_manifest.csv")

    lo, hi = pose_start + offset, pose_end + offset
    if lidar.get("source", "laz") == "bag":
        ds = BagScanDataset(cfg["paths"]["bag_dir"], cfg["run"]["lidar_topic"],
                            deskewer=deskewer, extrinsic=build_lidar_to_body(cfg),
                            frame_start=lo, frame_end=hi)
    else:
        ds = LazScanDataset(manifest, frame_start=lo, frame_end=hi)

    print(f"[partial_map] {cfg['run']['name']}: poses {pose_start}..{pose_end} "
          f"(bag frames {lo}..{hi}), voxel {voxel} m")

    global_map = o3d.geometry.PointCloud()
    pending = []

    def _down(points):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        return np.asarray(pc.voxel_down_sample(voxel).points)

    def _compact():
        nonlocal pending
        if not pending:
            return
        global_map.points.extend(o3d.utility.Vector3dVector(np.vstack(pending)))
        global_map.points = global_map.voxel_down_sample(voxel).points
        pending = []

    for k, scan in enumerate(tqdm(ds.iter_scans(), total=hi - lo + 1, desc="rebuilding")):
        i = pose_start + k
        t_s, pts, ptimes = scan[0], scan[1], scan[2]
        if abs(t_s - poses["timestamp"].iloc[i]) > 1e-3:
            raise SystemExit(f"pose {i}: scan time {t_s:.6f} != pose time "
                             f"{poses['timestamp'].iloc[i]:.6f} -- misaligned.")
        if len(pts) == 0:
            continue
        d = np.linalg.norm(pts, axis=1)
        gate = (d >= min_r) & (d <= max_r)
        pts = pts[gate]
        ptimes = np.asarray(ptimes)[gate] if len(ptimes) else ptimes
        if len(pts) == 0:
            continue
        if not imu_deskew and i > 0 and len(ptimes):
            pts = _deskew_with_delta(pts, ptimes, np.linalg.inv(T[i - 1]) @ T[i])
        pending.append(_down((T[i][:3, :3] @ pts.T).T + T[i][:3, 3]))
        if len(pending) >= 100:
            _compact()
    _compact()

    out = out_dir / f"map_local_{suffix}.pcd"
    o3d.io.write_point_cloud(str(out), global_map)
    print(f"[partial_map] {len(global_map.points)} points -> {out} "
          f"({out.stat().st_size / 1e6:.1f} MB)")
    if height:
        h = out_dir / f"map_local_{suffix}_height.pcd"
        colorize_by_height(out, h)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/config_test1.yaml")
    ap.add_argument("--poses", nargs=2, type=int, required=True,
                    metavar=("START", "END"))
    ap.add_argument("--suffix", required=True, help="output name suffix")
    ap.add_argument("--map-voxel-size", type=float, default=None)
    ap.add_argument("--no-height", action="store_true")
    args = ap.parse_args()
    build(args.config, args.poses[0], args.poses[1], args.suffix,
          args.map_voxel_size, height=not args.no_height)


if __name__ == "__main__":
    main()
