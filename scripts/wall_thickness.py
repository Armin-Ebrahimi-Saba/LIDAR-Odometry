#!/usr/bin/env python3
"""Cross-section through a revisited wall, with each pass drawn in its own colour.

Answers "shouldn't the walls be doubled, since there is no loop closure?" -- yes,
and this shows by how much. Two things hide it in `map_local.pcd`:

  * that map is voxel-filtered at `map_voxel_size` (0.7 m for Test1), which is
    coarser than the offset we are looking for, so the filter merges the two
    passes into one surface by construction;
  * a whole-site top-down render puts ~0.7 m at a couple of pixels.

So this rebuilds the neighbourhood at a fine voxel, keeps the two passes
separate, and plots a top-down slice plus a histogram of the along-normal
separation between them.

    python scripts/wall_thickness.py --pass-a 0 2781 --pass-b 4239 6723
"""
import argparse
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tqdm import tqdm

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sensys_slam.frames import build_lidar_to_body        # noqa: E402
from sensys_slam.lidar_io import BagScanDataset, LazScanDataset  # noqa: E402
from rebuild_map import _pose_matrices, _deskew_with_delta       # noqa: E402

A_COLOR = "#2a78d6"      # categorical slot 1 -- first pass
B_COLOR = "#eb6834"      # categorical slot 6 -- second pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/config_test1.yaml")
    ap.add_argument("--pass-a", nargs=2, type=int, required=True)
    ap.add_argument("--pass-b", nargs=2, type=int, required=True)
    ap.add_argument("--half-width", type=float, default=16.0,
                    help="half-size of the square window about the revisit point")
    ap.add_argument("--stride", type=int, default=2)
    ap.add_argument("--voxel", type=float, default=0.08,
                    help="fine voxel -- must be well under the offset being shown")
    ap.add_argument("--z-band", nargs=2, type=float, default=(-1.5, 0.5))
    ap.add_argument("--output", default=None)
    args = ap.parse_args()

    import open3d as o3d
    from scipy.spatial import cKDTree

    cfg = yaml.safe_load(Path(args.config).read_text())
    out_dir = Path(cfg["paths"]["output_dir"])
    out_path = Path(args.output) if args.output else out_dir / "wall_thickness.png"
    poses = pd.read_csv(out_dir / "poses_local.csv")
    T = _pose_matrices(poses)

    # Tightest revisit: where the two passes came closest in the pose track.
    A_xy = poses.iloc[args.pass_a[0]:args.pass_a[1] + 1][["x", "y"]].values
    B_xy = poses.iloc[args.pass_b[0]:args.pass_b[1] + 1][["x", "y"]].values
    dist, _ = cKDTree(A_xy).query(B_xy)
    j = int(np.argmin(dist))
    cx, cy = B_xy[j]
    cz = float(poses.iloc[args.pass_b[0] + j]["z"])
    print(f"[wall] revisit centre ({cx:.1f}, {cy:.1f}), pose separation "
          f"{dist[j]:.2f} m; window +-{args.half_width} m")

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
    lo = min(args.pass_a[0], args.pass_b[0]) + offset
    hi = max(args.pass_a[1], args.pass_b[1]) + offset
    if lidar.get("source", "laz") == "bag":
        ds = BagScanDataset(cfg["paths"]["bag_dir"], cfg["run"]["lidar_topic"],
                            deskewer=deskewer, extrinsic=build_lidar_to_body(cfg),
                            frame_start=lo, frame_end=hi)
    else:
        ds = LazScanDataset(manifest, frame_start=lo, frame_end=hi)

    hw, (zlo, zhi) = args.half_width, args.z_band
    buckets = {"A": [], "B": []}
    for k, scan in enumerate(tqdm(ds.iter_scans(), total=hi - lo + 1, desc="replaying")):
        i = lo + k - offset
        pts, ptimes = scan[1], scan[2]
        in_a = args.pass_a[0] <= i <= args.pass_a[1]
        in_b = args.pass_b[0] <= i <= args.pass_b[1]
        if not (in_a or in_b) or (i % args.stride) or len(pts) == 0:
            continue
        # Only scans taken near the window can see into it.
        if abs(T[i][0, 3] - cx) > hw + max_r or abs(T[i][1, 3] - cy) > hw + max_r:
            continue
        d = np.linalg.norm(pts, axis=1)
        gate = (d >= min_r) & (d <= max_r)
        pts, ptimes = pts[gate], (np.asarray(ptimes)[gate] if len(ptimes) else ptimes)
        if len(pts) == 0:
            continue
        if not imu_deskew and i > 0 and len(ptimes):
            pts = _deskew_with_delta(pts, ptimes, np.linalg.inv(T[i - 1]) @ T[i])
        w = (T[i][:3, :3] @ pts.T).T + T[i][:3, 3]
        keep = ((np.abs(w[:, 0] - cx) < hw) & (np.abs(w[:, 1] - cy) < hw)
                & (w[:, 2] - cz > zlo) & (w[:, 2] - cz < zhi))
        if keep.any():
            buckets["A" if in_a else "B"].append(w[keep])

    def _cloud(key):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(np.vstack(buckets[key]))
        return np.asarray(pc.voxel_down_sample(args.voxel).points)

    A, B = _cloud("A"), _cloud("B")
    print(f"[wall] pass A {len(A)} pts, pass B {len(B)} pts at {args.voxel} m voxel")

    # Separation between the two surfaces, restricted to points that actually
    # have a counterpart (2 m) so non-overlapping clutter is excluded.
    dAB, _ = cKDTree(A).query(B)
    sep = dAB[dAB < 2.0]

    fig, axes = plt.subplots(1, 2, figsize=(13, 6),
                             gridspec_kw={"width_ratios": [1.15, 1]})
    axes[0].scatter(A[:, 0], A[:, 1], s=0.6, c=A_COLOR, lw=0, label="first pass")
    axes[0].scatter(B[:, 0], B[:, 1], s=0.6, c=B_COLOR, lw=0, alpha=0.75,
                    label="second pass")
    axes[0].set_aspect("equal")
    axes[0].set_xlabel("x [m, SLAM local frame]"); axes[0].set_ylabel("y [m]")
    axes[0].set_title(f"Top-down slice, {zlo:+.1f} to {zhi:+.1f} m about the sensor\n"
                      f"voxel {args.voxel} m (map_local.pcd uses "
                      f"{cfg['kiss_icp']['mapping']['map_voxel_size']} m)", fontsize=10)
    leg = axes[0].legend(frameon=False, fontsize=9, markerscale=14)
    for t, c in zip(leg.get_texts(), (A_COLOR, B_COLOR)):
        t.set_color(c)

    axes[1].hist(sep, bins=120, range=(0, 2.0), color=B_COLOR, lw=0)
    for v, lbl, ls in ((np.median(sep), f"median {np.median(sep):.2f} m", "-"),
                       (np.percentile(sep, 90), f"p90 {np.percentile(sep, 90):.2f} m", "--")):
        axes[1].axvline(v, color="#3d3d3a", lw=1.2, ls=ls)
        axes[1].annotate(lbl, (v, 0.94), xycoords=("data", "axes fraction"),
                         fontsize=9, color="#3d3d3a", rotation=90,
                         ha="right", va="top")
    axes[1].set_xlabel("distance from a second-pass point to the nearest first-pass point [m]")
    axes[1].set_ylabel("points")
    axes[1].set_title("Surface separation between the two passes", fontsize=10)
    for side in ("top", "right"):
        axes[1].spines[side].set_visible(False)

    fig.suptitle(f"{cfg['run']['name']} -- is the wall doubled? "
                 f"(no loop closure between the passes)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[wall] separation median {np.median(sep):.3f}  p90 "
          f"{np.percentile(sep, 90):.3f} m  wrote {out_path}")


if __name__ == "__main__":
    main()
