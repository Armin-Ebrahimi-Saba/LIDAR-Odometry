"""Plot the GNSS ground-truth trajectory with its `eph` sampled every N seconds.

Purely a ground-truth view -- it uses the GT's own clock and needs no pose
matching, so nothing here depends on `alignment.time_match`.

`eph` is a magnitude, so it gets a sequential (single-hue, light->dark) encoding
carried by the track itself: the trajectory is drawn as per-segment coloured
line, at the ground truth's full ~5 Hz resolution, so where the GNSS solution
degrades is read straight off the path.

    python scripts/plot_gt_eph_map.py            # uses configs/config_test1.yaml
    # options: --config <path>  --output <path>
"""
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.collections import LineCollection

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sensys_slam.geo import geodetic_to_enu
from sensys_slam.groundtruth import load_ground_truth_for_run

# Sequential blue ramp, steps 100 -> 700 (light = low eph = confident).
# Step 100 is dropped: at marker size it recedes into the white surface.
EPH_RAMP = ["#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
PATH_COLOR = "#c9c8c2"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/config_test1.yaml")
    ap.add_argument("--output", default=None)
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    out_dir = Path(cfg["paths"]["output_dir"])
    out_path = Path(args.output) if args.output else out_dir / "gt_eph_map.png"

    gt = load_ground_truth_for_run(cfg)
    enu = geodetic_to_enu(gt["lat"].values, gt["lon"].values, gt["alt"].values,
                          float(gt["lat"].iloc[0]), float(gt["lon"].iloc[0]),
                          float(gt["alt"].iloc[0]))
    t = np.asarray(gt["timestamp"].values, float)
    t = t - t[0]
    eph = np.asarray(gt["eph"].values, float)

    xy = enu[:, :2]
    cmap = LinearSegmentedColormap.from_list("eph", EPH_RAMP)
    # Clip the ramp at the 98th percentile: a lone multi-metre GNSS excursion
    # (Test2 peaks at 11.6 m against a 0.66 m median) would otherwise compress
    # the entire run into the palest two steps. The colourbar is drawn with
    # extend="max" so the clipped tail is declared rather than hidden.
    hi = float(np.nanpercentile(eph, 98))
    clipped = float(np.nanmax(eph)) > hi
    norm = Normalize(vmin=float(np.nanmin(eph)), vmax=hi)

    # Size the canvas from the track's own aspect ratio -- the routes differ in
    # shape between runs (Test1 is a tall corridor, Test2 nearly square), and a
    # fixed figsize leaves one of them stranded in whitespace.
    span_e = max(xy[:, 0].ptp(), 1.0)
    span_n = max(xy[:, 1].ptp(), 1.0)
    h = 9.0
    w = float(np.clip(h * span_e / span_n, 4.0, 11.0)) + 2.2   # + colourbar/labels
    fig, ax = plt.subplots(figsize=(w, h))
    # One coloured segment per consecutive GT pair; a segment takes the mean eph
    # of its two endpoints so the colour is centred on the segment, not lagging.
    segs = np.stack([xy[:-1], xy[1:]], axis=1)
    lc = LineCollection(segs, cmap=cmap, norm=norm, capstyle="round",
                        linewidths=4.5, zorder=2)
    lc.set_array(0.5 * (eph[:-1] + eph[1:]))
    ax.add_collection(lc)
    # A recessive casing under the track keeps thin/pale stretches locatable.
    ax.plot(xy[:, 0], xy[:, 1], color=PATH_COLOR, lw=6.5, solid_capstyle="round",
            zorder=1, label="GNSS ground-truth track (colour = eph)")
    ax.autoscale_view()

    # Direct labels only on the extremes -- never a number on every point.
    for j in (int(np.nanargmin(eph)), int(np.nanargmax(eph))):
        ax.annotate(f"{eph[j]:.2f} m", xy[j],
                    textcoords="offset points", xytext=(9, 7), fontsize=8.5,
                    color="#3d3d3a",
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.85),
                    arrowprops=dict(arrowstyle="-", color="#8a8980", lw=0.6,
                                    shrinkA=0, shrinkB=4))

    ax.set_aspect("equal")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title(f"{cfg['run']['name']} -- GNSS ground-truth track\n"
                 f"coloured by horizontal uncertainty (eph)\n"
                 f"median {np.nanmedian(eph):.2f} m, max {np.nanmax(eph):.2f} m "
                 f"({len(eph)} samples)", fontsize=11)
    ax.grid(color="0.92", lw=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.legend(frameon=False, fontsize=8, loc="best")

    cb = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax,
                      fraction=0.05, pad=0.02, shrink=0.6,
                      extend="max" if clipped else "neither")
    lbl = "eph -- GNSS horizontal uncertainty, 1$\\sigma$ [m]"
    if clipped:
        lbl += f"\n(scale clipped at p98; peak {np.nanmax(eph):.2f} m)"
    cb.set_label(lbl, fontsize=9)
    cb.ax.tick_params(labelsize=8)
    cb.outline.set_visible(False)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[gt-eph-map] {len(eph)} GT samples  wrote {out_path}")


if __name__ == "__main__":
    main()
