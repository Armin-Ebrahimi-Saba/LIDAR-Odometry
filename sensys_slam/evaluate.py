"""Evaluate the aligned SLAM trajectory against the independent GNSS ground
truth: absolute positional error over time, RMSE, and a trajectory + error
plot.

Note this re-matches timestamps and computes error over the *entire*
ground-truth series, independent of which points were used to fit the
alignment in sensys_slam.align -- so this is a genuine accuracy check, not a
restatement of the alignment fit quality.
"""
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .geo import geodetic_to_enu
from .align import nearest_time_match


def evaluate_against_ground_truth(
    traj_latlon_df: pd.DataFrame, gt_df: pd.DataFrame, ref_origin, cfg: dict, output_dir: str
) -> dict:
    lat0, lon0, alt0 = ref_origin
    gt_enu = geodetic_to_enu(
        gt_df["lat"].values, gt_df["lon"].values, gt_df["alt"].values, lat0, lon0, alt0
    )

    max_diff = cfg.get("alignment", {}).get("max_time_diff_s", 0.15)
    q_idx, r_idx = nearest_time_match(
        traj_latlon_df["timestamp"].values, gt_df["timestamp"].values, max_diff
    )
    if len(q_idx) == 0:
        raise RuntimeError(
            "No timestamp matches found between the trajectory and ground "
            "truth for evaluation -- check time windows/epochs."
        )

    est = traj_latlon_df[["x_enu", "y_enu", "z_enu"]].values[q_idx]
    gt = gt_enu[r_idx]
    err = np.linalg.norm(est - gt, axis=1)

    rmse = float(np.sqrt(np.mean(err**2)))
    mean_err = float(np.mean(err))
    max_err = float(np.max(err))

    t_matched = traj_latlon_df["timestamp"].values[q_idx]
    t_rel = t_matched - t_matched[0]

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    axes[0].plot(gt[:, 0], gt[:, 1], label="GNSS ground truth", linewidth=2)
    axes[0].plot(est[:, 0], est[:, 1], label="Aligned LiDAR odometry", linewidth=1.2, alpha=0.85)
    axes[0].set_xlabel("East [m]")
    axes[0].set_ylabel("North [m]")
    axes[0].set_title("Trajectory (local ENU)")
    axes[0].legend()
    axes[0].axis("equal")

    axes[1].plot(t_rel, err)
    axes[1].set_xlabel("Time since start [s]")
    axes[1].set_ylabel("Position error [m]")
    axes[1].set_title(f"Absolute error -- RMSE = {rmse:.3f} m")

    fig.suptitle(cfg.get("evaluation", {}).get("plot_title", "LiDAR Odometry vs GNSS Ground Truth"))
    fig.tight_layout()

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_path = out_dir / "error_evaluation.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)

    metrics = {
        "rmse_m": rmse,
        "mean_error_m": mean_err,
        "max_error_m": max_err,
        "n_matched": int(len(err)),
    }
    metrics_path = out_dir / "error_metrics.csv"
    pd.DataFrame([metrics]).to_csv(metrics_path, index=False)

    print(f"[evaluate] RMSE={rmse:.3f} m  mean={mean_err:.3f} m  max={max_err:.3f} m  (n={len(err)})")
    print(f"[evaluate] wrote {plot_path}")
    print(f"[evaluate] wrote {metrics_path}")
    return metrics
