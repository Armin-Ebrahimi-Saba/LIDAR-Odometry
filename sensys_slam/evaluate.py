"""Evaluate the georeferenced SLAM trajectory against the GNSS ground truth:
absolute positional error over time, RMSE, and a trajectory + error plot.

This re-matches timestamps and computes error over the entire ground-truth
series, independent of which points were used to fit the alignment -- a genuine
accuracy check, not a restatement of the alignment fit quality.
"""
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .geo import geodetic_to_enu
from .align import nearest_time_match


def evaluate_against_ground_truth(traj_latlon_df, gt_df, ref_origin, cfg, output_dir) -> dict:
    lat0, lon0, alt0 = ref_origin
    gt_enu = geodetic_to_enu(gt_df["lat"].values, gt_df["lon"].values,
                             gt_df["alt"].values, lat0, lon0, alt0)

    max_diff = cfg.get("alignment", {}).get("max_time_diff_s", 0.15)
    q_idx, r_idx = nearest_time_match(traj_latlon_df["timestamp"].values,
                                      gt_df["timestamp"].values, max_diff)
    if len(q_idx) == 0:
        raise RuntimeError("No timestamp matches for evaluation -- check windows/epochs.")

    est = traj_latlon_df[["x_enu", "y_enu", "z_enu"]].values[q_idx]
    gt = gt_enu[r_idx]
    err = np.linalg.norm(est - gt, axis=1)

    rmse = float(np.sqrt(np.mean(err ** 2)))
    metrics = {
        "rmse_m": rmse,
        "mean_error_m": float(np.mean(err)),
        "max_error_m": float(np.max(err)),
        "n_matched": int(len(err)),
    }

    t_rel = traj_latlon_df["timestamp"].values[q_idx]
    t_rel = t_rel - t_rel[0]

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    # Two distinct, solid-colour trajectories; time is conveyed by the markers.
    axes[0].plot(gt[:, 0], gt[:, 1], color="tab:blue", lw=1.6, label="GNSS ground truth", zorder=1)
    axes[0].plot(est[:, 0], est[:, 1], color="tab:orange", lw=1.3, alpha=0.9,
                 label="LiDAR odometry", zorder=1)

    # A time "sign" every `time_tick_s` seconds (configurable), labelled on BOTH
    # trajectories in their own colour so the time is visible on each.
    tick_s = float(cfg.get("evaluation", {}).get("time_tick_s", 10.0))
    if tick_s and tick_s > 0 and t_rel[-1] > 0:
        for tt in np.arange(0.0, t_rel[-1] + 1e-9, tick_s):
            j = int(np.argmin(np.abs(t_rel - tt)))
            axes[0].scatter([gt[j, 0]], [gt[j, 1]], color="tab:blue", edgecolors="k",
                            s=40, linewidths=0.6, zorder=4)
            axes[0].scatter([est[j, 0]], [est[j, 1]], color="tab:orange", edgecolors="k",
                            marker="s", s=40, linewidths=0.6, zorder=4)
            axes[0].annotate(f"{tt:.0f}s", (gt[j, 0], gt[j, 1]), textcoords="offset points",
                             xytext=(4, 4), fontsize=8, color="tab:blue")
            axes[0].annotate(f"{tt:.0f}s", (est[j, 0], est[j, 1]), textcoords="offset points",
                             xytext=(4, -10), fontsize=8, color="tab:orange")
        axes[0].plot([], [], "ko", mfc="none", label=f"every {tick_s:.0f}s")

    axes[0].set_xlabel("East [m]"); axes[0].set_ylabel("North [m]")
    axes[0].set_title("Trajectory (local ENU)"); axes[0].legend(); axes[0].axis("equal")

    axes[1].plot(t_rel, err)
    axes[1].set_xlabel("Time since start [s]"); axes[1].set_ylabel("Position error [m]")
    axes[1].set_title(f"Absolute error -- RMSE = {rmse:.3f} m")

    fig.suptitle(cfg.get("evaluation", {}).get("plot_title", "LiDAR Odometry vs GNSS"))
    fig.tight_layout()

    out = Path(output_dir); out.mkdir(parents=True, exist_ok=True)
    plot_path = out / "error_evaluation.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)

    pd.DataFrame([metrics]).to_csv(out / "error_metrics.csv", index=False)
    print(f"[evaluate] RMSE={rmse:.3f} m  mean={metrics['mean_error_m']:.3f} m  "
          f"max={metrics['max_error_m']:.3f} m  (n={len(err)})")
    print(f"[evaluate] wrote {plot_path}")
    return metrics
