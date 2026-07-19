#!/usr/bin/env python3
"""Rebuild `map_local.pcd` from the existing `poses_local.csv` -- no ICP.

Why this exists
---------------
The map is just "transform each scan by its stored pose and accumulate", while
the poses are what cost the ICP. Re-tuning `kiss_icp.mapping.map_voxel_size`
therefore does not need a full odometry re-run: this script replays the bag
against the poses already on disk, which is a fraction of the cost.

CRITICAL -- frame consistency
-----------------------------
A pose maps *sensor frame -> world*, so the scans replayed here MUST be in the
same frame as the scans that produced the poses. That means `lidar.body_frame`,
`lidar.imu_deskew` and `kiss_icp.data.{min,max}_range` must still match the run
that wrote `poses_local.csv`. Change any of them and you need a real odometry
re-run, not this script -- the timestamp check below catches a differing scan
*count*, but it cannot detect a differing scan *frame*.

Deskew note: when `imu_deskew` is on the dataset already returns motion-
compensated clouds. Otherwise each sweep is deskewed with the *measured* pose
delta (T_prev^-1 @ T_curr) -- KISS-ICP itself can only use the constant-velocity
*prediction*, so this is if anything slightly sharper.

Usage:
    python scripts/rebuild_map.py                      # uses config's map_voxel_size
    python scripts/rebuild_map.py --map-voxel-size 0.2
    python scripts/rebuild_map.py --output outputs/test1/map_dense.pcd
"""
import argparse
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import yaml
from scipy.spatial.transform import Rotation, Slerp
from tqdm import tqdm

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sensys_slam.frames import build_lidar_to_body  # noqa: E402
from sensys_slam.lidar_io import BagScanDataset, LazScanDataset  # noqa: E402


def _pose_matrices(poses: pd.DataFrame) -> np.ndarray:
    """(N, 4, 4) sensor->world transforms from the poses CSV."""
    T = np.zeros((len(poses), 4, 4))
    T[:, :3, :3] = Rotation.from_quat(poses[["qx", "qy", "qz", "qw"]].values).as_matrix()
    T[:, :3, 3] = poses[["x", "y", "z"]].values
    T[:, 3, 3] = 1.0
    return T


def _deskew_with_delta(points, point_times, delta):
    """Deskew a sweep using the measured pose delta over the scan interval.

    `point_times` is normalised to [0, 1] across the sweep; each point is moved
    to the sweep-end pose by interpolating `delta` (SLERP on rotation, linear on
    translation) and applying the *residual* motion still to come after it.
    """
    if len(point_times) == 0 or len(points) == 0:
        return points
    key_rot = Rotation.from_matrix(np.stack([np.eye(3), delta[:3, :3]]))
    slerp = Slerp([0.0, 1.0], key_rot)

    uniq, inv = np.unique(point_times, return_inverse=True)
    R_s = slerp(uniq).as_matrix()                       # (M,3,3) motion up to s
    t_s = uniq[:, None] * delta[:3, 3]                  # (M,3)
    # Point measured at s, expressed at the sweep-end frame: T_delta^-1 @ T_s.
    R_e, t_e = delta[:3, :3].T, -delta[:3, :3].T @ delta[:3, 3]
    R_rel = np.einsum("ij,mjk->mik", R_e, R_s)          # (M,3,3)
    t_rel = t_s @ R_e.T + t_e                           # (M,3)
    return np.einsum("mij,mj->mi", R_rel[inv], points) + t_rel[inv]


def rebuild_map(config_path, map_voxel_size=None, output=None, deskew=True,
                color="none", cmap="turbo", intensity_percentile=(2.0, 98.0),
                frame_start=None, frame_end=None):
    import open3d as o3d

    want_intensity = color == "intensity"

    cfg = yaml.safe_load(Path(config_path).read_text())
    out_dir = Path(cfg["paths"]["output_dir"])
    poses_path = out_dir / "poses_local.csv"
    if not poses_path.exists():
        raise SystemExit(f"{poses_path} not found -- run the odometry stage first.")

    poses = pd.read_csv(poses_path)
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

    run_cfg = cfg.get("run", {})
    # Frame range overrides (e.g. from run_pipeline's --frames) take precedence
    # over the config, so the rebuilt map matches the poses that were produced.
    if frame_start is None:
        frame_start = int(run_cfg.get("frame_start") or 0)
    if frame_end is None:
        frame_end = run_cfg.get("frame_end")
        if frame_end is None and run_cfg.get("max_frames"):
            frame_end = frame_start + int(run_cfg["max_frames"]) - 1

    if lidar.get("source", "laz") == "bag":
        ds = BagScanDataset(cfg["paths"]["bag_dir"], cfg["run"]["lidar_topic"],
                            deskewer=deskewer, extrinsic=build_lidar_to_body(cfg),
                            frame_start=frame_start, frame_end=frame_end,
                            with_intensity=want_intensity)
    else:
        if want_intensity:
            raise SystemExit("color=intensity needs lidar.source: bag "
                             "(the .laz exports carry no intensity).")
        ds = LazScanDataset(pd.read_csv(out_dir / "scan_manifest.csv"),
                            frame_start=frame_start, frame_end=frame_end)

    if len(ds) != len(poses):
        raise SystemExit(
            f"{len(ds)} scans but {len(poses)} poses -- the frame range in the "
            f"config no longer matches the run that wrote {poses_path.name}.")

    print(f"[rebuild_map] {len(poses)} poses, map_voxel_size={voxel} m, "
          f"range=[{min_r}, {max_r}] m, imu_deskew={imu_deskew}, color={color}")

    global_map = o3d.geometry.PointCloud()
    pending = []          # list of (points_ds, gray_ds-or-None)

    # Intensity is carried through voxel downsampling as a grayscale colour so
    # open3d averages it together with the point coordinates. We normalise it to
    # [0, 1] against a robust reference taken from the first scan that has it
    # (kept fixed thereafter so scans stay comparable); the final colormap is
    # applied once at the end after a second, contrast-stretching normalise.
    intensity_ref = {"scale": None}

    def _gray(intensity):
        if intensity_ref["scale"] is None:
            hi = float(np.percentile(intensity, 99.0))
            lo = float(np.percentile(intensity, 1.0))
            intensity_ref["scale"] = (lo, max(hi - lo, 1e-6))
        lo, span = intensity_ref["scale"]
        return np.clip((intensity - lo) / span, 0.0, 1.0)

    def _down(points, gray):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        if gray is not None:
            pc.colors = o3d.utility.Vector3dVector(np.repeat(gray[:, None], 3, axis=1))
        d = pc.voxel_down_sample(voxel)
        dp = np.asarray(d.points)
        return dp, (np.asarray(d.colors)[:, 0] if gray is not None else None)

    def _compact():
        nonlocal pending
        if not pending:
            return
        pts_all = np.vstack([p for p, _ in pending])
        global_map.points.extend(o3d.utility.Vector3dVector(pts_all))
        if want_intensity:
            gray_all = np.concatenate([g for _, g in pending])
            global_map.colors.extend(
                o3d.utility.Vector3dVector(np.repeat(gray_all[:, None], 3, axis=1)))
        d = global_map.voxel_down_sample(voxel)
        global_map.points = d.points
        global_map.colors = d.colors
        pending = []

    for i, scan in enumerate(
            tqdm(ds.iter_scans(), total=len(ds), desc="rebuilding map")):
        t_s, pts, ptimes = scan[0], scan[1], scan[2]
        intensity = scan[3] if want_intensity else None
        if abs(t_s - poses["timestamp"].iloc[i]) > 1e-3:
            raise SystemExit(
                f"scan {i} time {t_s:.6f} != pose time "
                f"{poses['timestamp'].iloc[i]:.6f} -- poses/scans are misaligned.")
        if len(pts) == 0:
            continue
        # Same range gate KISS-ICP applied before these poses were estimated.
        d = np.linalg.norm(pts, axis=1)
        gate = (d >= min_r) & (d <= max_r)
        pts = pts[gate]
        ptimes = np.asarray(ptimes)[gate] if len(ptimes) else ptimes
        if intensity is not None and len(intensity):
            intensity = np.asarray(intensity)[gate]
        if len(pts) == 0:
            continue
        if deskew and not imu_deskew and i > 0 and len(ptimes):
            pts = _deskew_with_delta(pts, ptimes, np.linalg.inv(T[i - 1]) @ T[i])

        world = (T[i][:3, :3] @ pts.T).T + T[i][:3, 3]
        gray = _gray(intensity) if (want_intensity and len(intensity)) else None
        pending.append(_down(world, gray))
        if len(pending) >= 100:
            _compact()
    _compact()

    if want_intensity:
        from matplotlib import colormaps
        g = np.asarray(global_map.colors)[:, 0]
        lo, hi = np.percentile(g, intensity_percentile)
        norm = np.clip((g - lo) / (hi - lo + 1e-12), 0.0, 1.0)
        global_map.colors = o3d.utility.Vector3dVector(colormaps[cmap](norm)[:, :3])

    out = Path(output) if output else out_dir / "map_local.pcd"
    o3d.io.write_point_cloud(str(out), global_map)
    n = len(global_map.points)
    print(f"[rebuild_map] {n} map points -> {out} "
          f"({out.stat().st_size / 1e6:.1f} MB)")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--map-voxel-size", type=float, default=None,
                    help="override kiss_icp.mapping.map_voxel_size (m)")
    ap.add_argument("--no-deskew", action="store_true",
                    help="skip pose-delta deskew of each sweep")
    ap.add_argument("--color", choices=["none", "intensity"], default="none",
                    help="colour the map by the LiDAR's per-point return strength")
    ap.add_argument("--cmap", default="turbo", help="matplotlib colormap for --color intensity")
    ap.add_argument("--output", default=None, help="PCD output path")
    args = ap.parse_args()
    rebuild_map(args.config, args.map_voxel_size, args.output, not args.no_deskew,
                color=args.color, cmap=args.cmap)


if __name__ == "__main__":
    main()
