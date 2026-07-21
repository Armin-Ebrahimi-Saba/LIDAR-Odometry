#!/usr/bin/env python3
"""Measure accumulated drift by how well two passes over the same ground agree.

Why this is not circular
------------------------
`map_local.pcd` looks sharp partly by construction: KISS-ICP registers each scan
*against the accumulated local map*, so consecutive scans are actively pulled
into agreement. Nothing in the pipeline, however, forces the *second* traversal
of a corridor to land on top of the first -- the local map is pruned to
`max_range`, so by the time the vehicle comes back the earlier pass is no longer
in the registration reference. Any offset between the two passes is therefore
drift the frontend accumulated in between, and it is measured against the LiDAR
itself rather than against GNSS.

Method
------
1. Split the run into two passes by frame index (`--pass-a`, `--pass-b`), chosen
   from the pose track (positions <`--revisit-radius` m apart, far apart in
   index). Rebuild each pass into its own cloud from `poses_local.csv`, using the
   same range gate and deskew as the odometry run (`rebuild_map` conventions).
2. Restrict both to their spatial overlap and to a height band around the
   sensor, which drops canopy: foliage genuinely differs between passes and
   would swamp a geometric comparison.
3. Report nearest-neighbour distance B->A, then run point-to-plane ICP of B onto
   A. **The translation of that ICP transform is the drift estimate**: it is the
   rigid offset needed to bring the second pass back onto the first.
4. CONTROL: the same statistic computed within a single pass (even vs odd
   scans), which shares no such gap. That is the noise floor -- sensor noise,
   voxel quantisation, viewpoint change -- and the revisit number is only
   meaningful to the extent it exceeds it.

    python scripts/revisit_consistency.py --pass-a 0 2781 --pass-b 4239 6723
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


def _stats(name, d):
    print(f"    {name:<28} median {np.median(d):6.3f}  mean {np.mean(d):6.3f}  "
          f"p90 {np.percentile(d, 90):6.3f}  p99 {np.percentile(d, 99):6.3f} m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/config_test1.yaml")
    ap.add_argument("--pass-a", nargs=2, type=int, required=True, metavar=("START", "END"))
    ap.add_argument("--pass-b", nargs=2, type=int, required=True, metavar=("START", "END"))
    ap.add_argument("--stride", type=int, default=4,
                    help="use every Nth scan (default 4) -- density is ample")
    ap.add_argument("--voxel", type=float, default=0.25)
    ap.add_argument("--z-band", nargs=2, type=float, default=(-2.5, 1.5),
                    help="keep points in this height band about the sensor "
                         "(default -2.5 1.5) -- excludes canopy")
    ap.add_argument("--max-nn", type=float, default=2.0,
                    help="discard NN pairs beyond this (non-overlap / clutter)")
    args = ap.parse_args()

    import open3d as o3d

    cfg = yaml.safe_load(Path(args.config).read_text())
    out_dir = Path(cfg["paths"]["output_dir"])
    poses = pd.read_csv(out_dir / "poses_local.csv")
    T = _pose_matrices(poses)

    data = cfg.get("kiss_icp", {}).get("data", {})
    min_r, max_r = float(data.get("min_range", 0.0)), float(data.get("max_range", 100.0))
    lidar = cfg.get("lidar", {})
    imu_deskew = bool(lidar.get("imu_deskew", False))
    deskewer = None
    if imu_deskew:
        from sensys_slam.attitude import load_attitude_deskewer, PX4_ATTITUDE_TOPIC
        deskewer = load_attitude_deskewer(
            cfg["paths"]["bag_dir"], lidar.get("attitude_topic", PX4_ATTITUDE_TOPIC))

    # Pass indices are POSE indices. The odometry run may have started partway
    # into the bag (`--frames`), so pose i is bag frame i + offset; recover the
    # offset from the manifest rather than trusting the config's frame_start.
    manifest = pd.read_csv(out_dir / "scan_manifest.csv")
    offset = int(np.argmin(np.abs(manifest["timestamp"].values
                                  - poses["timestamp"].iloc[0])))
    if abs(manifest["timestamp"].values[offset] - poses["timestamp"].iloc[0]) > 1e-3:
        raise SystemExit("cannot locate pose[0] in scan_manifest.csv")
    print(f"[revisit] poses 0..{len(poses)-1} == bag frames "
          f"{offset}..{offset + len(poses) - 1}")

    lo = min(args.pass_a[0], args.pass_b[0]) + offset
    hi = max(args.pass_a[1], args.pass_b[1]) + offset
    if lidar.get("source", "laz") == "bag":
        ds = BagScanDataset(cfg["paths"]["bag_dir"], cfg["run"]["lidar_topic"],
                            deskewer=deskewer, extrinsic=build_lidar_to_body(cfg),
                            frame_start=lo, frame_end=hi)
    else:
        ds = LazScanDataset(pd.read_csv(out_dir / "scan_manifest.csv"),
                            frame_start=lo, frame_end=hi)

    # Buckets: pass A, pass B, and A split even/odd for the control.
    buckets = {"A": [], "B": [], "A_even": [], "A_odd": []}
    zlo, zhi = args.z_band

    for k, scan in enumerate(tqdm(ds.iter_scans(), total=hi - lo + 1, desc="replaying")):
        i = lo + k - offset                            # index into poses
        t_s, pts, ptimes = scan[0], scan[1], scan[2]
        if abs(t_s - poses["timestamp"].iloc[i]) > 1e-3:
            raise SystemExit(f"scan {i} time mismatch -- poses/scans misaligned.")
        in_a = args.pass_a[0] <= i <= args.pass_a[1]
        in_b = args.pass_b[0] <= i <= args.pass_b[1]
        if not (in_a or in_b) or (i % args.stride) or len(pts) == 0:
            continue
        d = np.linalg.norm(pts, axis=1)
        gate = (d >= min_r) & (d <= max_r)
        pts, ptimes = pts[gate], (np.asarray(ptimes)[gate] if len(ptimes) else ptimes)
        if len(pts) == 0:
            continue
        if not imu_deskew and i > 0 and len(ptimes):
            pts = _deskew_with_delta(pts, ptimes, np.linalg.inv(T[i - 1]) @ T[i])
        world = (T[i][:3, :3] @ pts.T).T + T[i][:3, 3]
        # Height band is relative to the sensor at that instant, so it tracks
        # the slope of the route rather than a global plane.
        band = (world[:, 2] - T[i][2, 3] > zlo) & (world[:, 2] - T[i][2, 3] < zhi)
        world = world[band]
        if not len(world):
            continue
        buckets["A" if in_a else "B"].append(world)
        if in_a:
            buckets["A_even" if (i // args.stride) % 2 == 0 else "A_odd"].append(world)

    def _cloud(key):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(np.vstack(buckets[key]))
        return pc.voxel_down_sample(args.voxel)

    A, B, Ae, Ao = (_cloud(k) for k in ("A", "B", "A_even", "A_odd"))
    print(f"\n[revisit] pass A {len(A.points)} pts, pass B {len(B.points)} pts "
          f"(voxel {args.voxel} m, stride {args.stride}, z-band {zlo}..{zhi} m)")

    def _nn(src, dst):
        d = np.asarray(src.compute_point_cloud_distance(dst))
        return d[d < args.max_nn]

    print("\n  Nearest-neighbour surface distance:")
    ctrl = _nn(Ae, Ao)
    _stats("CONTROL within-pass (A/A)", ctrl)
    before = _nn(B, A)
    _stats("REVISIT B->A, as estimated", before)

    # Point-to-plane ICP: the rigid offset that puts pass B back onto pass A.
    A.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))
    reg = o3d.pipelines.registration.registration_icp(
        B, A, args.max_nn, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    from scipy.spatial.transform import Rotation
    M = reg.transformation
    rpy = Rotation.from_matrix(M[:3, :3]).as_euler("xyz", degrees=True)
    # The matrix's translation column is referenced to the WORLD ORIGIN, so a
    # fraction of a degree of rotation about an origin hundreds of metres away
    # shows up there as metres of bogus offset. The physically meaningful
    # quantity is how far the transform actually moves the overlapping points,
    # so evaluate the displacement field on the cloud itself.
    Bp = np.asarray(B.points)
    disp = (Bp @ M[:3, :3].T + M[:3, 3]) - Bp
    c = Bp.mean(axis=0)
    t = (M[:3, :3] @ c + M[:3, 3]) - c          # displacement at the centroid
    dmag = np.linalg.norm(disp, axis=1)
    after = _nn(o3d.geometry.PointCloud(B).transform(M), A)
    _stats("REVISIT B->A, after ICP", after)

    print(f"\n  DRIFT between passes (ICP B->A):")
    print(f"    at overlap centroid  E {t[0]:+.3f}  N {t[1]:+.3f}  U {t[2]:+.3f} m"
          f"   |horizontal| {np.hypot(t[0], t[1]):.3f} m  |3D| {np.linalg.norm(t):.3f} m")
    print(f"    displacement over the overlap: median {np.median(dmag):.3f}  "
          f"p10 {np.percentile(dmag, 10):.3f}  p90 {np.percentile(dmag, 90):.3f}  "
          f"max {dmag.max():.3f} m")
    print(f"    rotation     roll {rpy[0]:+.3f}  pitch {rpy[1]:+.3f}  "
          f"yaw {rpy[2]:+.3f} deg")
    print(f"    (raw matrix translation, referenced to the world origin and NOT "
          f"the drift: {M[0,3]:+.2f} {M[1,3]:+.2f} {M[2,3]:+.2f} m)")
    print(f"    ICP fitness {reg.fitness:.3f}, inlier RMSE {reg.inlier_rmse:.3f} m")
    print(f"\n  Revisit median exceeds the within-pass floor by "
          f"{np.median(before) - np.median(ctrl):+.3f} m before ICP, "
          f"{np.median(after) - np.median(ctrl):+.3f} m after.")


if __name__ == "__main__":
    main()
