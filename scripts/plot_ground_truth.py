#!/usr/bin/env python3
"""Plot the GNSS ground-truth trajectory from xtrack_global_position_t12.csv.

By default plots the configured run window (e.g. Test1); pass --full to plot the
entire file (both datasets) with the Test1/Test2 windows highlighted. Two
panels: a time-coloured ENU bird's-eye trajectory and the ENU components over
time.

Usage:
    python scripts/plot_ground_truth.py                 # current run (/configs/config_test1.yaml)
    python scripts/plot_ground_truth.py --full          # whole file, both runs
    python scripts/plot_ground_truth.py --output path.png
    python scripts/plot_ground_truth.py --times "100 300 500"   # seconds since start
    python scripts/plot_ground_truth.py --frames "1000 1500"    # LiDAR frame indices
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
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sensys_slam.geo import geodetic_to_enu  # noqa: E402

# Run windows (Unix epoch seconds), inventory report 5.2.
TEST1 = (1780397390.972, 1780398213.329)
TEST2 = (1780398327.532, 1780398929.389)


def _time_col(df):
    return "timestamp_sample" if "timestamp_sample" in df.columns else "timestamp"


def _parse_floats(spec):
    """Parse 'a b ; c, d' -> [a, b, c, d] of floats."""
    import re
    return [float(tok) for tok in re.split(r"[;,\s]+", (spec or "").strip()) if tok]


def _lidar_frame_times(bag_dir, topic):
    """Absolute bag-record times (s) of every message on `topic`, in order.

    Used to translate LiDAR frame indices (e.g. 1000, 1500) into wall-clock
    times so they can be marked on the GNSS trajectory.
    """
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.LATEST)
    times = []
    with AnyReader([Path(bag_dir)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise SystemExit(f"Topic '{topic}' not found in {bag_dir}.")
        for _c, t_ns, _raw in reader.messages(connections=conns):
            times.append(t_ns * 1e-9)
    return np.asarray(times)


def plot_ground_truth(config_path, full=False, output=None, times=None, frames=None):
    cfg = yaml.safe_load(Path(config_path).read_text())
    df = pd.read_csv(cfg["paths"]["gnss_csv"])
    tcol = _time_col(df)

    if full:
        title_prefix = "Complete GNSS ground truth"
        default_out = "outputs/gnss_ground_truth_full.png"
    else:
        t0, t1 = cfg["run"]["start_time"], cfg["run"]["end_time"]
        df = df[(df[tcol] >= t0) & (df[tcol] <= t1)].reset_index(drop=True)
        title_prefix = f"{cfg['run'].get('name', 'run')} GNSS ground truth"
        default_out = "outputs/gnss_ground_truth_run.png"
    if df.empty:
        raise SystemExit("No ground-truth samples in the selected window.")

    t = df[tcol].values
    lat0, lon0, alt0 = float(df.lat.iloc[0]), float(df.lon.iloc[0]), float(df.alt.iloc[0])
    enu = geodetic_to_enu(df.lat.values, df.lon.values, df.alt.values, lat0, lon0, alt0)
    E, N, U = enu[:, 0], enu[:, 1], enu[:, 2]
    trel = t - t[0]
    path2d = float(np.sum(np.linalg.norm(np.diff(enu[:, :2], axis=0), axis=1)))
    disp = float(np.linalg.norm(enu[-1, :2] - enu[0, :2]))

    fig, ax = plt.subplots(1, 2, figsize=(15, 6))
    ax[0].plot(E, N, color="0.6", lw=0.6, zorder=0)
    sc = ax[0].scatter(E, N, c=trel, cmap="viridis", s=5)
    if full:
        for win, col, lab in [(TEST1, "tab:red", "Test1"), (TEST2, "tab:orange", "Test2")]:
            m = (t >= win[0]) & (t <= win[1])
            ax[0].plot(E[m], N[m], color=col, lw=1.5, label=f"{lab} window")
    ax[0].scatter([E[0]], [N[0]], c="k", s=70, marker="o", label="start", zorder=5)
    ax[0].scatter([E[-1]], [N[-1]], c="r", s=70, marker="X", label="end", zorder=5)

    # Build the list of (absolute_time, label) marks to draw on the trajectory:
    #   --times  : seconds since run start, or absolute Unix epoch (auto-detected)
    #   --frames : LiDAR frame indices, translated to bag time via /ouster/points
    marks = []
    for tv in (times or []):
        t_abs = tv if tv > 1e6 else t[0] + tv
        marks.append((t_abs, f"{tv:.0f}s" if tv <= 1e6 else f"{t_abs - t[0]:.0f}s"))
    if frames:
        ft = _lidar_frame_times(cfg["paths"]["bag_dir"],
                                cfg["run"].get("lidar_topic", "/ouster/points"))
        for fi in frames:
            fi = int(fi)
            if fi < 0 or fi >= len(ft):
                print(f"[plot_ground_truth] WARNING: frame {fi} out of range "
                      f"(0..{len(ft) - 1}); skipped.")
                continue
            marks.append((ft[fi], f"f{fi}"))

    # The GNSS df is already cropped to the run window, so t[0]..t[-1] is the
    # valid marking span. Warn (don't silently snap to the end) if a mark lies
    # outside it -- that was the "no marks appear" footgun for frame-sized values.
    time_marks = []
    for k, (t_abs, label) in enumerate(marks):
        if t_abs < t[0] - 1e-6 or t_abs > t[-1] + 1e-6:
            print(f"[plot_ground_truth] WARNING: mark '{label}' at {t_abs - t[0]:+.1f}s "
                  f"is outside the run window (0..{t[-1] - t[0]:.0f}s); not drawn.")
            continue
        j = int(np.argmin(np.abs(t - t_abs)))
        time_marks.append((t[j] - t[0], enu[j, 0], enu[j, 1]))
        ax[0].scatter([enu[j, 0]], [enu[j, 1]], c="cyan", marker="D", s=90,
                      edgecolor="k", linewidth=0.6, zorder=6,
                      label="marks" if k == 0 else None)
        ax[0].annotate(label, (enu[j, 0], enu[j, 1]),
                       textcoords="offset points", xytext=(6, -12), fontsize=9, color="teal")
        print(f"[plot_ground_truth] {label} @ {t[j]-t[0]:.1f}s -> "
              f"ENU=({enu[j,0]:.1f}, {enu[j,1]:.1f}) m")

    ax[0].set_xlabel("East [m]"); ax[0].set_ylabel("North [m]")
    ax[0].set_title(f"{title_prefix} (ENU)"); ax[0].axis("equal"); ax[0].legend()
    fig.colorbar(sc, ax=ax[0], label="time since start [s]")

    ax[1].plot(trel, E, label="East")
    ax[1].plot(trel, N, label="North")
    ax[1].plot(trel, U, label="Up")
    if full:
        ax[1].axvspan(TEST1[0] - t[0], TEST1[1] - t[0], color="tab:red", alpha=0.15, label="Test1")
        ax[1].axvspan(TEST2[0] - t[0], TEST2[1] - t[0], color="tab:orange", alpha=0.15, label="Test2")
    for k, (trel_mark, _e, _n) in enumerate(time_marks):
        ax[1].axvline(trel_mark, color="cyan", ls=":", lw=1.2,
                      label="marks" if k == 0 else None)
    ax[1].set_xlabel("time since start [s]"); ax[1].set_ylabel("position [m]")
    ax[1].set_title("ENU components over time"); ax[1].legend()

    fig.suptitle(f"{title_prefix}  --  {len(df)} samples, {trel[-1]:.0f} s, "
                 f"2D path {path2d:.0f} m, displacement {disp:.0f} m")
    fig.tight_layout()

    out = Path(output) if output else Path(default_out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"[plot_ground_truth] {len(df)} samples, {trel[-1]:.1f}s, "
          f"2D path {path2d:.1f}m, displacement {disp:.1f}m")
    print(f"[plot_ground_truth] wrote {out}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="/configs/config_test1.yaml")
    ap.add_argument("--full", action="store_true", help="plot the whole file (both datasets)")
    ap.add_argument("--times", default=None, metavar='"t1 t2"',
                    help="mark GNSS position at these times: seconds since run start, "
                         "or absolute Unix epoch (auto-detected)")
    ap.add_argument("--frames", default=None, metavar='"f1 f2"',
                    help="mark GNSS position at these LiDAR frame indices "
                         "(e.g. \"1000 1500\"); translated to time via /ouster/points")
    ap.add_argument("--output", default=None)
    args = ap.parse_args()
    plot_ground_truth(args.config, args.full, args.output,
                      times=_parse_floats(args.times),
                      frames=_parse_floats(args.frames))


if __name__ == "__main__":
    main()
