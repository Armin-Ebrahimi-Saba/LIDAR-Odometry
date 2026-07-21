#!/usr/bin/env python3
"""End-to-end LiDAR positioning pipeline for the Sensys dataset.

Stages (run with --stage, or 'all' to run everything in order):
  1. fetch      -- download the run's dataset, the GNSS ground truth and the PX4
                    message definitions into data/ (skipped if already present)
  2. timestamps -- pair .laz scans with bag-recorded timestamps
  3. odometry   -- KISS-ICP odometry -> local poses + 3D map,
                    seeded at the first GNSS ground-truth point
  4. align      -- anchored SE(3) georeference to GNSS -> lat/lon
  5. velocity   -- finite-difference the trajectory -> NED velocity
  6. evaluate   -- RMSE + error-over-time plot vs. ground truth
  7. map        -- render odometry + GNSS on OpenStreetMap (HTML)
  8. map3d      -- render the SLAM 3D point-cloud map (interactive HTML)
  9. colormaps  -- colour the 3D map: map_local_height.pcd (by elevation) and
                    map_local_intensity.pcd (by the LiDAR's per-point return)

`--stage X` runs stage X and every stage after it (they chain off each other),
so `--stage align` re-does align -> velocity -> evaluate -> map -> map3d.
`--stage all` (default) runs the whole pipeline.

Usage:
    python run_pipeline.py --config /configs/config_test1.yaml
    python run_pipeline.py --config /configs/config_test1.yaml --stage align   # align onward
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from sensys_slam.fetch import ensure_data
from sensys_slam.timestamps import build_scan_manifest
from sensys_slam.lidar_io import LazScanDataset, BagScanDataset
from sensys_slam.odometry import run_odometry
from sensys_slam.groundtruth import load_ground_truth_for_run
from sensys_slam.align import align_and_georeference
from sensys_slam.velocity import compute_ned_velocity
from sensys_slam.evaluate import evaluate_against_ground_truth

ALL_STAGES = ["fetch", "timestamps", "odometry", "align", "velocity", "evaluate",
              "map", "map3d", "colormaps"]


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def gt_first_point(cfg: dict):
    """ENU tangent origin = first GNSS ground-truth sample of the run."""
    gt = load_ground_truth_for_run(cfg)
    return (float(gt["lat"].iloc[0]), float(gt["lon"].iloc[0]), float(gt["alt"].iloc[0]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="/configs/config_test1.yaml")
    parser.add_argument("--stage", default="all", choices=["all"] + ALL_STAGES,
                        help="run this stage AND all stages after it (e.g. --stage "
                             "align runs align, velocity, evaluate). 'all' runs everything.")
    parser.add_argument("--frames", nargs=2, type=int, metavar=("START", "END"), default=None,
                        help="closed range of scan indices to process, e.g. --frames 1000 2000 "
                             "(overrides run.frame_start/frame_end in the config)")
    parser.add_argument("--data-root", default="data",
                        help="where the fetch stage downloads/extracts the data (default: data)")
    parser.add_argument("--keep-zips", action="store_true",
                        help="keep the downloaded dataset zip after extracting it "
                             "(off by default -- they are ~16-21 GB each)")
    args = parser.parse_args()

    cfg = load_config(args.config)
    out_dir = Path(cfg["paths"]["output_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = out_dir / "scan_manifest.csv"
    origin_path = out_dir / "alignment_origin.yaml"

    # Frame range to process. Precedence: --frames > run.frame_start/frame_end >
    # legacy run.max_frames (first N scans).
    run_cfg = cfg.get("run", {})
    frame_start = int(run_cfg.get("frame_start") or 0)
    frame_end = run_cfg.get("frame_end")
    if frame_end is None and run_cfg.get("max_frames"):
        frame_end = frame_start + int(run_cfg["max_frames"]) - 1
    if args.frames:
        frame_start, frame_end = args.frames
    if frame_end is not None and frame_end < frame_start:
        raise SystemExit(f"frame range END ({frame_end}) < START ({frame_start}).")

    # Run the requested stage and every stage after it (a stage depends on the
    # outputs of the ones before, so "from here on" is the useful default).
    stages = (ALL_STAGES if args.stage == "all"
              else ALL_STAGES[ALL_STAGES.index(args.stage):])

    if "fetch" in stages:
        print("\n== Stage 1: fetching input data (skipped where already present) ==")
        ensure_data(cfg, args.data_root, args.keep_zips)

    if "timestamps" in stages:
        print("\n== Stage 2: building LiDAR scan timestamp manifest ==")
        build_scan_manifest(
            bag_dir=cfg["paths"]["bag_dir"],
            laz_dir=cfg["paths"]["laz_dir"],
            topic=cfg["run"]["lidar_topic"],
            out_csv=str(manifest_path),
        )

    if "odometry" in stages:
        print("\n== Stage 3: running KISS-ICP odometry/SLAM ==")
        print(f"[odometry] frame range: {frame_start}..{frame_end if frame_end is not None else 'end'}")
        # Seed the world frame at the first GNSS ground-truth point: the ENU
        # tangent origin is that sample, so the odometry starts at ENU (0,0,0).
        ref_origin = gt_first_point(cfg)
        print(f"[odometry] seeding start at first GT point lat={ref_origin[0]:.7f} "
              f"lon={ref_origin[1]:.7f} alt={ref_origin[2]:.2f}")

        source = cfg.get("lidar", {}).get("source", "laz")
        if source == "bag":
            print("[odometry] LiDAR source: bag (/ouster/points)")
            # Optional: express scans in the Pixhawk FRD body frame (so they
            # share the PX4 attitude's convention) -- see sensys_slam.frames.
            from sensys_slam.frames import build_lidar_to_body
            extrinsic = build_lidar_to_body(cfg)
            if extrinsic is not None:
                print("[odometry] body frame: Pixhawk FRD (LiDAR->body extrinsic applied)")
            # Optional: deskew each sweep with measured PX4 attitude instead of
            # KISS-ICP's constant-velocity model. When on, the cloud is already
            # rotation-compensated, so KISS-ICP's own deskew is forced off.
            deskewer = None
            if cfg.get("lidar", {}).get("imu_deskew", False):
                from sensys_slam.attitude import load_attitude_deskewer, PX4_ATTITUDE_TOPIC
                att_topic = cfg.get("lidar", {}).get("attitude_topic", PX4_ATTITUDE_TOPIC)
                print(f"[odometry] IMU-attitude deskew ON (attitude: {att_topic})")
                deskewer = load_attitude_deskewer(cfg["paths"]["bag_dir"], att_topic)
                cfg.setdefault("kiss_icp", {}).setdefault("data", {})["deskew"] = False
            dataset = BagScanDataset(cfg["paths"]["bag_dir"], cfg["run"]["lidar_topic"],
                                     deskewer=deskewer, extrinsic=extrinsic,
                                     frame_start=frame_start, frame_end=frame_end)
        elif source == "laz":
            print("[odometry] LiDAR source: laz files")
            dataset = LazScanDataset(pd.read_csv(manifest_path),
                                     frame_start=frame_start, frame_end=frame_end)
        else:
            raise ValueError(f"Unknown lidar.source '{source}' (expected 'laz' or 'bag').")

        run_odometry(dataset, cfg, str(out_dir), initial_pose=np.eye(4))
        with open(origin_path, "w") as f:
            yaml.dump({"lat0": ref_origin[0], "lon0": ref_origin[1], "alt0": ref_origin[2]}, f)

    if "align" in stages:
        print("\n== Stage 4: aligning SLAM trajectory to GNSS ground truth ==")
        poses_df = pd.read_csv(out_dir / "poses_local.csv")
        gt_df = load_ground_truth_for_run(cfg)
        ref_origin = None
        if origin_path.exists():
            o = yaml.safe_load(origin_path.read_text())
            ref_origin = (o["lat0"], o["lon0"], o["alt0"])
        traj_df, ref_origin, fit_rmse, _ = align_and_georeference(poses_df, gt_df, cfg, ref_origin)
        traj_df.to_csv(out_dir / "trajectory_latlon.csv", index=False)
        with open(origin_path, "w") as f:
            yaml.dump({"lat0": float(ref_origin[0]), "lon0": float(ref_origin[1]),
                       "alt0": float(ref_origin[2]), "alignment_fit_rmse_m": fit_rmse}, f)
        print(f"[align] alignment fit RMSE (matched points) = {fit_rmse:.3f} m")
        print(f"[align] wrote {out_dir / 'trajectory_latlon.csv'}")

    if "velocity" in stages:
        print("\n== Stage 5: computing NED velocity ==")
        traj_df = pd.read_csv(out_dir / "trajectory_latlon.csv")
        vel_cfg = cfg.get("velocity", {})
        vel_df = compute_ned_velocity(
            traj_df,
            smooth_window=vel_cfg.get("smooth_window", 11),
            smooth_polyorder=vel_cfg.get("smooth_polyorder", 2),
        )
        vel_df[["timestamp", "vel_n_m_s", "vel_e_m_s", "vel_d_m_s"]].to_csv(
            out_dir / "velocity_ned.csv", index=False)
        vel_df.to_csv(out_dir / "trajectory_latlon_with_velocity.csv", index=False)
        print(f"[velocity] wrote {out_dir / 'velocity_ned.csv'}")

    if "evaluate" in stages:
        print("\n== Stage 6: evaluating against ground truth (RMSE + plot) ==")
        traj_df = pd.read_csv(out_dir / "trajectory_latlon.csv")
        gt_df = load_ground_truth_for_run(cfg)
        o = yaml.safe_load(origin_path.read_text())
        ref_origin = (o["lat0"], o["lon0"], o["alt0"])
        evaluate_against_ground_truth(traj_df, gt_df, ref_origin, cfg, str(out_dir))

    if any(s in stages for s in ("map", "map3d", "colormaps")):
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parent / "scripts"))

    if "map" in stages:
        print("\n== Stage 7: rendering odometry + GNSS on OpenStreetMap ==")
        from plot_map import make_map
        make_map(args.config)

    if "map3d" in stages:
        print("\n== Stage 8: rendering SLAM 3D point-cloud map ==")
        from plot_map3d import make_map3d
        make_map3d(args.config)

    if "colormaps" in stages:
        print("\n== Stage 9: building coloured 3D point-cloud maps ==")
        from colorize_map import colorize_by_height
        from rebuild_map import rebuild_map
        map_path = out_dir / "map_local.pcd"
        if not map_path.exists():
            raise SystemExit(f"{map_path} not found -- run the odometry stage first.")
        # Height: instant post-process of the accumulated map (no bag read).
        colorize_by_height(map_path, out_dir / "map_local_height.pcd")
        # Intensity: re-read the bag to attach the LiDAR's per-point return. Pass
        # the same frame range odometry used so the map matches poses_local.csv.
        rebuild_map(args.config, output=str(out_dir / "map_local_intensity.pcd"),
                    color="intensity", frame_start=frame_start, frame_end=frame_end)

    print("\nDone. Outputs in:", out_dir.resolve())


if __name__ == "__main__":
    main()
