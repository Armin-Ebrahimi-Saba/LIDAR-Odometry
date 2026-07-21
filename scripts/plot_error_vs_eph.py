"""Plot the absolute SLAM position error and the GNSS `eph` on one chart.

Both quantities are metres, so they share a single y-axis: the eph curve is the
ground truth's own horizontal uncertainty, i.e. the noise floor below which an
error is not meaningfully distinguishable from GNSS wander.

    python scripts/plot_error_vs_eph.py            # uses configs/config_test1.yaml
    # options: --config <path>  --output <path>
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sensys_slam.geo import geodetic_to_enu
from sensys_slam.groundtruth import load_ground_truth_for_run
from sensys_slam.align import match_poses_to_gt

ERR_COLOR = "#2a78d6"   # categorical slot 1
EPH_COLOR = "#eb6834"   # categorical slot 6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/config_test1.yaml")
    ap.add_argument("--output", default=None)
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    out_dir = Path(cfg["paths"]["output_dir"])
    out_path = Path(args.output) if args.output else out_dir / "error_vs_eph.png"

    traj_df = pd.read_csv(out_dir / "trajectory_latlon.csv")
    gt_df = load_ground_truth_for_run(cfg)
    o = yaml.safe_load((out_dir / "alignment_origin.yaml").read_text())

    gt_enu = geodetic_to_enu(gt_df["lat"].values, gt_df["lon"].values,
                             gt_df["alt"].values, o["lat0"], o["lon0"], o["alt0"])
    q_idx, r_idx = match_poses_to_gt(traj_df, gt_df, cfg)
    err = np.linalg.norm(traj_df[["x_enu", "y_enu", "z_enu"]].values[q_idx]
                         - gt_enu[r_idx], axis=1)
    eph = np.asarray(gt_df["eph"].values, float)[r_idx]

    t = traj_df["timestamp"].values[q_idx]
    t = t - t[0]

    # NOTE ON THE X-AXIS: with alignment.time_match "arclength"/"proportional"
    # the two streams do NOT share a clock -- poses are paired to GT samples by
    # distance travelled, and the matched GT sample's own timestamp can be well
    # over a minute away from the pose timestamp. So this is not "eph over time":
    # each x is a *pair*, and the eph curve is the uncertainty of the GT sample
    # that particular error was measured against. We report the skew so the
    # distinction is on the chart rather than left to the reader.
    gt_t = np.asarray(gt_df["timestamp"].values, float)[r_idx]
    skew = (gt_t - gt_t[0]) - t
    paired = cfg.get("alignment", {}).get("time_match", "absolute") != "absolute"

    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(t, err, color=ERR_COLOR, lw=2.0, label="Absolute position error")
    ax.plot(t, eph, color=EPH_COLOR, lw=2.0, label="GNSS eph (ground-truth uncertainty)")

    # Direct labels at the right edge, so identity is never colour-alone.
    ax.annotate("error", (t[-1], err[-1]), xytext=(6, 0), textcoords="offset points",
                color=ERR_COLOR, fontsize=9, va="center")
    ax.annotate("eph", (t[-1], eph[-1]), xytext=(6, 0), textcoords="offset points",
                color=EPH_COLOR, fontsize=9, va="center")

    ax.set_xlabel("LiDAR time since start [s]")
    ax.set_ylabel("Metres")
    ax.set_title(f"{cfg['run']['name']} -- absolute error vs. GNSS eph  "
                 f"(RMSE {np.sqrt(np.mean(err ** 2)):.2f} m, "
                 f"median eph {np.nanmedian(eph):.2f} m)")
    if paired:
        ax.text(0.0, -0.20,
                f"x = LiDAR clock. eph is that of the GT sample each error was "
                f"measured against, paired by "
                f"'{cfg['alignment']['time_match']}', not by time -- the matched "
                f"GT sample's own clock differs by a median of "
                f"{np.median(skew):+.0f} s (range {skew.min():+.0f} to "
                f"{skew.max():+.0f} s). Features in the two curves at the same x "
                f"are NOT simultaneous.",
                transform=ax.transAxes, fontsize=7.5, color="0.35",
                va="top", wrap=True)
    ax.set_xlim(t[0], t[-1] * 1.03)
    ax.set_ylim(bottom=0)
    ax.grid(axis="y", color="0.9", lw=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.legend(frameon=False, loc="upper left")

    fig.tight_layout(rect=(0, 0.08, 1, 1))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[error-vs-eph] n={len(err)}  wrote {out_path}")


if __name__ == "__main__":
    main()
