#!/usr/bin/env python3
"""Plot two LiDAR scans top-down, raw vs de-registered, to show whether the
clouds carry ego-motion.

Left panel: the raw `/ouster/points` clouds (already in a fixed world frame --
they overlap, i.e. no apparent motion). Right panel: the same two scans after
de-registration back to the sensor frame (they separate by the platform's
motion). Also prints the nearest-neighbour overlap before/after.

Usage:
    python scripts/plot_scans.py 1 200                  # scans 1 and 200
    python scripts/plot_scans.py 3000 3200 --source gnss_attitude
    python scripts/plot_scans.py 1 1000 --output outputs/scans_1_1000.png
"""
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sensys_slam.lidar_io import BagScanDataset  # noqa: E402


def _load_deregisterer(cfg, source):
    if source == "gnss":
        from sensys_slam.deregister import load_gnss_deregisterer
        return load_gnss_deregisterer(cfg)
    if source == "gnss_attitude":
        from sensys_slam.deregister import load_gnss_attitude_deregisterer
        return load_gnss_attitude_deregisterer(cfg)
    from sensys_slam.deregister import load_deregisterer
    return load_deregisterer(cfg["paths"]["bag_dir"])


def _crop_box(P, box):
    """Keep points with |x| AND |y| in [box[0], box[1]] (z unrestricted) -- the
    four corner regions, i.e. both the [lo,hi] and [-hi,-lo] brackets."""
    lo, hi = box
    ax, ay = np.abs(P[:, 0]), np.abs(P[:, 1])
    m = (ax >= lo) & (ax <= hi) & (ay >= lo) & (ay <= hi)
    return P[m]


def plot_scans(config_path, a, b, source="gnss", output=None, stride=3, box=None):
    cfg = yaml.safe_load(Path(config_path).read_text())
    ds = BagScanDataset(cfg["paths"]["bag_dir"], cfg["run"]["lidar_topic"], frame_end=b)
    keep = {}
    for i, (t, f, _pt) in enumerate(ds.iter_scans()):
        if i in (a, b):
            keep[i] = (t, f)
        if i > b:
            break
    if a not in keep or b not in keep:
        raise SystemExit(f"Could not read both scans {a} and {b} (only {sorted(keep)}).")
    (tA, wA), (tB, wB) = keep[a], keep[b]

    if box is not None:
        nA, nB = len(wA), len(wB)
        wA, wB = _crop_box(wA, box), _crop_box(wB, box)
        print(f"[plot_scans] box x,y in [{box[0]},{box[1]}]: "
              f"frame {a} {nA}->{len(wA)} pts, frame {b} {nB}->{len(wB)} pts")
        if len(wA) == 0 or len(wB) == 0:
            raise SystemExit("No points left after box crop -- widen --box.")

    dereg = _load_deregisterer(cfg, source)
    sA, sB = dereg.deregister(wA, tA), dereg.deregister(wB, tB)

    # Overlap: nearest-neighbour distance from scan b to scan a (lower = more
    # overlap). Reported as median NN and the fraction of points within 0.3 m.
    raw_nn = cKDTree(wA).query(wB)[0]
    dereg_nn = cKDTree(sA).query(sB)[0]
    raw_txt = f"overlap: median NN={np.median(raw_nn):.2f} m\nwithin 0.3 m={np.mean(raw_nn < 0.3):.0%}"
    dereg_txt = f"overlap: median NN={np.median(dereg_nn):.2f} m\nwithin 0.3 m={np.mean(dereg_nn < 0.3):.0%}"

    fig, ax = plt.subplots(1, 2, figsize=(15, 7))

    def scat(axis, P, c, lab):
        axis.scatter(P[::stride, 0], P[::stride, 1], s=1, c=c, alpha=0.4, label=lab)

    def overlap_box(axis, text):
        axis.text(0.02, 0.98, text, transform=axis.transAxes, va="top", ha="left",
                  fontsize=11, bbox=dict(boxstyle="round", fc="white", ec="0.5", alpha=0.85))

    scat(ax[0], wA, "tab:blue", f"scan {a}"); scat(ax[0], wB, "tab:red", f"scan {b}")
    ax[0].set_title(f"RAW world-frame clouds (scan {a} vs {b})\n-> overlap = no apparent motion")
    overlap_box(ax[0], raw_txt)
    scat(ax[1], sA, "tab:blue", f"scan {a}"); scat(ax[1], sB, "tab:red", f"scan {b}")
    ax[1].set_title(f"After de-registration ({source}) -> sensor frame\n-> offset = recovered motion")
    overlap_box(ax[1], dereg_txt)
    for axis in ax:
        axis.set_xlabel("x [m]"); axis.set_ylabel("y [m]")
        axis.axis("equal"); axis.legend(markerscale=6, loc="upper right")
    fig.suptitle(f"LiDAR scans {a} and {b}  (dt={tB - tA:.1f}s)")
    fig.tight_layout()

    if box is not None:
        # Boxes belong only on the raw panel: the crop is applied in the sensor
        # frame, so after de-registration the points no longer sit inside it.
        lo, hi, w = box[0], box[1], box[1] - box[0]
        corners = [(lo, lo), (lo, -hi), (-hi, lo), (-hi, -hi)]  # four |x|,|y| in [lo,hi] boxes
        for cx, cy in corners:
            ax[0].add_patch(Rectangle((cx, cy), w, w, fill=False, ls="--", ec="k"))

    suffix = f"_box{int(box[0])}-{int(box[1])}" if box is not None else ""
    out = Path(output) if output else Path(f"outputs/scans_{a}_vs_{b}{suffix}.png")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)

    print(f"[plot_scans] raw world  : NN median={np.median(raw_nn):.3f} m  frac<0.3m={np.mean(raw_nn < 0.3):.0%}")
    print(f"[plot_scans] de-reg ({source}): NN median={np.median(dereg_nn):.3f} m  frac<0.3m={np.mean(dereg_nn < 0.3):.0%}")
    print(f"[plot_scans] wrote {out}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("frame_a", type=int)
    ap.add_argument("frame_b", type=int)
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--source", default="gnss", choices=["gnss", "gnss_attitude", "px4"],
                    help="de-registration pose source for the right panel")
    ap.add_argument("--stride", type=int, default=3, help="plot every Nth point")
    ap.add_argument("--box", type=float, nargs=2, metavar=("MIN", "MAX"), default=None,
                    help="keep only points with x AND y in [MIN, MAX] (e.g. --box 5 20)")
    ap.add_argument("--output", default=None)
    args = ap.parse_args()
    plot_scans(args.config, args.frame_a, args.frame_b, args.source, args.output,
               args.stride, args.box)


if __name__ == "__main__":
    main()
