#!/usr/bin/env python3
"""End-to-end LiDAR positioning pipeline for the Sensys dataset.

Stages (run with --stage, or 'all' to run everything in order):
  1. timestamps -- pair .laz scans with bag-recorded timestamps
  2. odometry   -- run KISS-ICP odometry/SLAM -> local poses + 3D map
  3. align      -- SE(3)-align the local trajectory to GNSS ground truth,
                    re-express it as lat/lon
  4. velocity   -- finite-difference the aligned trajectory -> NED velocity
  5. evaluate   -- RMSE + error-over-time plot vs. ground truth

Usage:
    python run_pipeline.py --config config.yaml
    python run_pipeline.py --config config.yaml --stage odometry
"""
import argparse
from pathlib import Path

import pandas as pd
import yaml

from sensys_slam.timestamps import build_scan_manifest
from sensys_slam.lidar_io import LazScanDataset
from sensys_slam.odometry import run_odometry
from sensys_slam.groundtruth import load_ground_truth_for_run
from sensys_slam.align import align_and_georeference
from sensys_slam.velocity import compute_ned_velocity
from sensys_slam.evaluate import evaluate_against_ground_truth

ALL_STAGES = ["timestamps", "odometry", "align", "velocity", "evaluate"]


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--stage", default="all", choices=["all"] + ALL_STAGES)
    args = parser.parse_args()

    cfg = load_config(args.config)
    out_dir = Path(cfg["paths"]["output_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = out_dir / "scan_manifest.csv"

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
        manifest_df = pd.read_csv(manifest_path)
        dataset = LazScanDataset(manifest_df)
        run_odometry(dataset, cfg, str(out_dir))

    if "align" in stages:
        print("\n== Stage 3: aligning SLAM trajectory to GNSS ground truth ==")
        poses_df = pd.read_csv(out_dir / "poses_local.csv")
        gt_df = load_ground_truth_for_run(cfg)
        traj_df, ref_origin, fit_rmse, _ = align_and_georeference(poses_df, gt_df, cfg)
        traj_df.to_csv(out_dir / "trajectory_latlon.csv", index=False)
        with open(out_dir / "alignment_origin.yaml", "w") as f:
            yaml.dump(
                {
                    "lat0": float(ref_origin[0]),
                    "lon0": float(ref_origin[1]),
                    "alt0": float(ref_origin[2]),
                    "alignment_fit_rmse_m": fit_rmse,
                },
                f,
            )
        print(f"[align] alignment fit RMSE (on matched calibration points) = {fit_rmse:.3f} m")
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
            out_dir / "velocity_ned.csv", index=False
        )
        vel_df.to_csv(out_dir / "trajectory_latlon_with_velocity.csv", index=False)
        print(f"[velocity] wrote {out_dir / 'velocity_ned.csv'}")

    if "evaluate" in stages:
        print("\n== Stage 5: evaluating against ground truth (RMSE + plot) ==")
        traj_df = pd.read_csv(out_dir / "trajectory_latlon.csv")
        gt_df = load_ground_truth_for_run(cfg)
        with open(out_dir / "alignment_origin.yaml") as f:
            origin_cfg = yaml.safe_load(f)
        ref_origin = (origin_cfg["lat0"], origin_cfg["lon0"], origin_cfg["alt0"])
        evaluate_against_ground_truth(traj_df, gt_df, ref_origin, cfg, str(out_dir))

    print("\nDone. Outputs in:", out_dir.resolve())


if __name__ == "__main__":
    main()
