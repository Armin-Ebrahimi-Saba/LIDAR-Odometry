#!/usr/bin/env python3
"""Downstream georeferencing/evaluation pipeline for the Sensys dataset.

On this LIO-SAM branch the local trajectory is produced by LIO_SAM_6AXIS (see
LIO_SAM_NOTES.md and run_liosam_pipeline.sh), which writes
``outputs/<run>/poses_local.csv``. These stages are odometry-engine agnostic --
they only consume that CSV plus the GNSS ground truth:

  1. align     -- SE(3)-align the local trajectory to GNSS, re-express as lat/lon
  2. velocity  -- finite-difference the aligned trajectory -> NED velocity
  3. evaluate  -- RMSE + error-over-time plot vs. ground truth

Usage:
    python run_pipeline.py --config config_liosam.yaml --stage align
    python run_pipeline.py --config config_liosam.yaml            # all three
"""
import argparse
from pathlib import Path

import pandas as pd
import yaml

from sensys_slam.groundtruth import load_ground_truth_for_run
from sensys_slam.align import align_and_georeference
from sensys_slam.velocity import compute_ned_velocity
from sensys_slam.evaluate import evaluate_against_ground_truth

ALL_STAGES = ["align", "velocity", "evaluate"]


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config_liosam.yaml")
    parser.add_argument("--stage", default="all", choices=["all"] + ALL_STAGES)
    args = parser.parse_args()

    cfg = load_config(args.config)
    out_dir = Path(cfg["paths"]["output_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)

    stages = ALL_STAGES if args.stage == "all" else [args.stage]

    if "align" in stages:
        print("\n== Stage: aligning SLAM trajectory to GNSS ground truth ==")
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
        print("\n== Stage: computing NED velocity ==")
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
        print("\n== Stage: evaluating against ground truth (RMSE + plot) ==")
        traj_df = pd.read_csv(out_dir / "trajectory_latlon.csv")
        gt_df = load_ground_truth_for_run(cfg)
        with open(out_dir / "alignment_origin.yaml") as f:
            origin_cfg = yaml.safe_load(f)
        ref_origin = (origin_cfg["lat0"], origin_cfg["lon0"], origin_cfg["alt0"])
        evaluate_against_ground_truth(traj_df, gt_df, ref_origin, cfg, str(out_dir))

    print("\nDone. Outputs in:", out_dir.resolve())


if __name__ == "__main__":
    main()
