#!/usr/bin/env python3
"""End-to-end LiDAR positioning pipeline for the Sensys dataset.

Stages (run with --stage, or 'all' to run everything in order):
  1. timestamps -- pair .laz scans with bag-recorded timestamps
  2. odometry   -- from-scratch KISS-ICP odometry/SLAM -> local poses + 3D map,
                    seeded at the first GNSS ground-truth point
  3. align      -- start-anchored SE(3) georeference to GNSS -> lat/lon
  4. velocity   -- finite-difference the trajectory -> NED velocity
  5. evaluate   -- RMSE + error-over-time plot vs. ground truth

Usage:
    python run_pipeline.py --config config.yaml
    python run_pipeline.py --config config.yaml --stage odometry
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from sensys_slam.timestamps import build_scan_manifest
from sensys_slam.lidar_io import LazScanDataset, BagScanDataset
from sensys_slam.odometry import run_odometry
from sensys_slam.groundtruth import load_ground_truth_for_run
from sensys_slam.align import align_and_georeference
from sensys_slam.velocity import compute_ned_velocity
from sensys_slam.evaluate import evaluate_against_ground_truth

ALL_STAGES = ["timestamps", "odometry", "align", "velocity", "evaluate"]


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def gt_first_point(cfg: dict):
    """ENU tangent origin = first GNSS ground-truth sample of the run."""
    gt = load_ground_truth_for_run(cfg)
    return (float(gt["lat"].iloc[0]), float(gt["lon"].iloc[0]), float(gt["alt"].iloc[0]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--stage", default="all", choices=["all"] + ALL_STAGES)
    parser.add_argument("--frames", nargs=2, type=int, metavar=("START", "END"), default=None,
                        help="closed range of scan indices to process, e.g. --frames 1000 2000 "
                             "(overrides run.frame_start/frame_end in the config)")
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

    stages = ALL_STAGES if args.stage == "all" else [args.stage]

    if "timestamps" in stages:
        print("\n== Stage 1: building LiDAR scan timestamp manifest ==")
        build_scan_manifest(
            bag_dir=cfg["paths"]["bag_dir"],
            laz_dir=cfg["paths"]["laz_dir"],
            topic=cfg["run"]["lidar_topic"],
            out_csv=str(manifest_path),
        )

    if "odometry" in stages:
        print("\n== Stage 2: running KISS-ICP odometry/SLAM ==")
        print(f"[odometry] frame range: {frame_start}..{frame_end if frame_end is not None else 'end'}")
        # Seed the world frame at the first GNSS ground-truth point: the ENU
        # tangent origin is that sample, so the odometry starts at ENU (0,0,0).
        ref_origin = gt_first_point(cfg)
        print(f"[odometry] seeding start at first GT point lat={ref_origin[0]:.7f} "
              f"lon={ref_origin[1]:.7f} alt={ref_origin[2]:.2f}")

        source = cfg.get("lidar", {}).get("source", "laz")
        if source == "bag":
            print("[odometry] LiDAR source: bag (/ouster/points)")
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
                                     deskewer=deskewer,
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
        print("\n== Stage 3: aligning SLAM trajectory to GNSS ground truth ==")
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
        print("\n== Stage 4: computing NED velocity ==")
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
        print("\n== Stage 5: evaluating against ground truth (RMSE + plot) ==")
        traj_df = pd.read_csv(out_dir / "trajectory_latlon.csv")
        gt_df = load_ground_truth_for_run(cfg)
        o = yaml.safe_load(origin_path.read_text())
        ref_origin = (o["lat0"], o["lon0"], o["alt0"])
        evaluate_against_ground_truth(traj_df, gt_df, ref_origin, cfg, str(out_dir))

    print("\nDone. Outputs in:", out_dir.resolve())


if __name__ == "__main__":
    main()
