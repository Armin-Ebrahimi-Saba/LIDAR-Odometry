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
from .align import match_poses_to_gt, match_weights


def evaluate_against_ground_truth(traj_latlon_df, gt_df, ref_origin, cfg, output_dir) -> dict:
    lat0, lon0, alt0 = ref_origin
    gt_enu = geodetic_to_enu(gt_df["lat"].values, gt_df["lon"].values,
                             gt_df["alt"].values, lat0, lon0, alt0)

    q_idx, r_idx = match_poses_to_gt(traj_latlon_df, gt_df, cfg)
    if len(q_idx) == 0:
        raise RuntimeError("No timestamp matches for evaluation -- check windows/epochs.")

    est = traj_latlon_df[["x_enu", "y_enu", "z_enu"]].values[q_idx]
    gt = gt_enu[r_idx]
    err = np.linalg.norm(est - gt, axis=1)

    rmse = float(np.sqrt(np.mean(err ** 2)))
    # eph-weighted RMSE: down-weights errors measured against uncertain ground
    # truth (1/eph^2), so the metric is not inflated by the several-metre GNSS
    # wander during initialisation. Equals the raw RMSE when weighting is off.
    w = match_weights(gt_df, r_idx, cfg)
    rmse_w = (float(np.sqrt(np.average(err ** 2, weights=w))) if w is not None
              else rmse)
    metrics = {
        "rmse_m": rmse,
        "rmse_weighted_m": rmse_w,
        "mean_error_m": float(np.mean(err)),
        "max_error_m": float(np.max(err)),
        "n_matched": int(len(err)),
    }
    if "eph" in gt_df.columns:
        eph_matched = np.asarray(gt_df["eph"].values, float)[r_idx]
        metrics["eph_median_m"] = float(np.nanmedian(eph_matched))
        metrics["eph_max_m"] = float(np.nanmax(eph_matched))

    t_rel = traj_latlon_df["timestamp"].values[q_idx]
    t_rel = t_rel - t_rel[0]

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for a in axes:
        a.tick_params(axis="both", labelsize=7)
    # Two distinct, solid-colour trajectories; time is conveyed by the markers.
    axes[0].plot(gt[:, 0], gt[:, 1], color="tab:blue", lw=1.6, label="GNSS ground truth", zorder=1)
    axes[0].plot(est[:, 0], est[:, 1], color="tab:orange", lw=1.3, alpha=0.9,
                 label="LiDAR odometry", zorder=1)

    # A time "sign" every `time_tick_s` seconds (configurable), labelled on BOTH
    # trajectories in their own colour so the time is visible on each. Labels are
    # placed collision-aware: where the two trajectories nearly coincide (small
    # error) their two same-time labels are merged into one, and any label that
    # would land on top of an already-placed one is skipped, so nothing overlaps.
    tick_s = float(cfg.get("evaluation", {}).get("time_tick_s", 10.0))
    axes[0].axis("equal")
    if tick_s and tick_s > 0 and t_rel[-1] > 0:
        # Fix the axes' data limits/aspect first so display-space distances used
        # for de-cluttering match the saved figure.
        axes[0].autoscale(False)
        to_disp = axes[0].transData.transform
        placed = []            # display-space (x, y) of labels already drawn
        MERGE_PX, MIN_PX = 14.0, 20.0   # merge if closer than MERGE_PX; else keep MIN_PX apart
        bbox = dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.7)

        def _free(xy_disp):
            return all((xy_disp[0] - px[0]) ** 2 + (xy_disp[1] - px[1]) ** 2 > MIN_PX ** 2
                       for px in placed)

        def _label(pt, text, color, dy):
            d = to_disp(pt)
            if not _free(d):
                return False
            axes[0].annotate(text, pt, textcoords="offset points", xytext=(3, dy),
                             fontsize=8, color=color, zorder=6, bbox=bbox,
                             arrowprops=dict(arrowstyle="-", color=color, lw=0.4,
                                             shrinkA=0, shrinkB=1))
            placed.append(d)
            return True

        for tt in np.arange(0.0, t_rel[-1] + 1e-9, tick_s):
            j = int(np.argmin(np.abs(t_rel - tt)))
            axes[0].scatter([gt[j, 0]], [gt[j, 1]], color="tab:blue", edgecolors="k",
                            s=16, linewidths=0.5, zorder=4)
            axes[0].scatter([est[j, 0]], [est[j, 1]], color="tab:orange", edgecolors="k",
                            marker="s", s=16, linewidths=0.5, zorder=4)
            dg, de = to_disp(gt[j, :2]), to_disp(est[j, :2])
            coincident = (dg[0] - de[0]) ** 2 + (dg[1] - de[1]) ** 2 < MERGE_PX ** 2
            if coincident:
                # one merged label (black) at the midpoint of the two markers
                _label((gt[j, :2] + est[j, :2]) / 2, f"{tt:.0f}s", "k", 6)
            else:
                _label(gt[j, :2], f"{tt:.0f}s", "tab:blue", 6)
                _label(est[j, :2], f"{tt:.0f}s", "tab:orange", -9)
        axes[0].plot([], [], "ko", mfc="none", label=f"every {tick_s:.0f}s")

    axes[0].set_xlabel("East [m]"); axes[0].set_ylabel("North [m]")
    axes[0].set_title("Trajectory (local ENU)"); axes[0].legend()

    axes[1].plot(t_rel, err)
    axes[1].set_xlabel("Time since start [s]"); axes[1].set_ylabel("Position error [m]")
    title = f"Absolute error -- RMSE = {rmse:.3f} m"
    if w is not None:
        title += f"  (eph-weighted {rmse_w:.3f} m)"
    axes[1].set_title(title)

    fig.suptitle(cfg.get("evaluation", {}).get("plot_title", "LiDAR Odometry vs GNSS"))
    fig.tight_layout()

    out = Path(output_dir); out.mkdir(parents=True, exist_ok=True)
    plot_path = out / "error_evaluation.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)

    pd.DataFrame([metrics]).to_csv(out / "error_metrics.csv", index=False)
    wtxt = f"  eph-weighted RMSE={rmse_w:.3f} m" if w is not None else ""
    print(f"[evaluate] RMSE={rmse:.3f} m{wtxt}  mean={metrics['mean_error_m']:.3f} m  "
          f"max={metrics['max_error_m']:.3f} m  (n={len(err)})")
    if "eph_median_m" in metrics:
        print(f"[evaluate] GNSS eph over matched samples: median="
              f"{metrics['eph_median_m']:.2f} m  max={metrics['eph_max_m']:.2f} m")
    print(f"[evaluate] wrote {plot_path}")
    return metrics
