#!/usr/bin/env python3
"""Overlay IMU-heading vs GNSS-course arrows along the GNSS trajectory.

At every `--interval` seconds along the run, draw two arrows at the GNSS
position:
  * IMU heading  (blue)  -- where the platform points (PX4 /fmu/out/vehicle_attitude,
                            body-forward axis, NED->ENU)
  * GNSS course  (red)   -- direction of travel (finite-difference of the GNSS
                            track); skipped where the speed is below --min-speed
                            (course is meaningless when nearly stationary).

Where the two arrows diverge, the platform is pointing one way while moving
another (e.g. translated/pulled without turning).

Usage:
    python scripts/plot_headings.py                       # uses config.yaml
    python scripts/plot_headings.py --interval 10 --arrow-len 12
"""
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sensys_slam.groundtruth import load_ground_truth_for_run  # noqa: E402
from sensys_slam.geo import geodetic_to_enu  # noqa: E402
from sensys_slam.attitude import load_attitude_deskewer  # noqa: E402


def plot_headings(config_path, interval=20.0, arrow_len=10.0, min_speed=0.3, output=None):
    cfg = yaml.safe_load(Path(config_path).read_text())
    gt = load_ground_truth_for_run(cfg)
    lat0, lon0, alt0 = float(gt.lat.iloc[0]), float(gt.lon.iloc[0]), float(gt.alt.iloc[0])
    enu = geodetic_to_enu(gt.lat.values, gt.lon.values, gt.alt.values, lat0, lon0, alt0)
    t = gt.timestamp.values.astype(float)
    att = load_attitude_deskewer(cfg["paths"]["bag_dir"])

    def imu_dir(tt):  # body-forward unit vector in ENU (E, N)
        R = att._slerp([np.clip(tt, att.t0, att.t1)])[0].as_matrix()
        f = R @ np.array([1.0, 0.0, 0.0])          # NED (N, E, D)
        v = np.array([f[1], f[0]])                  # -> ENU (E, N)
        n = np.linalg.norm(v)
        return v / n if n else v

    def course_dir(tt, dt=1.5):                     # GNSS travel unit vector + speed
        i0 = int(np.argmin(np.abs(t - (tt - dt))))
        i1 = int(np.argmin(np.abs(t - (tt + dt))))
        d = enu[i1, :2] - enu[i0, :2]
        span = max(t[i1] - t[i0], 1e-6)
        spd = np.linalg.norm(d) / span
        n = np.linalg.norm(d)
        return (d / n if n else d), spd

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(enu[:, 0], enu[:, 1], color="0.6", lw=1.0, zorder=0, label="GNSS trajectory")
    ax.scatter([enu[0, 0]], [enu[0, 1]], c="k", s=70, marker="o", zorder=5, label="start")
    ax.scatter([enu[-1, 0]], [enu[-1, 1]], c="r", s=70, marker="X", zorder=5, label="end")

    for k, tt in enumerate(np.arange(t[0], t[-1] + 1e-9, interval)):
        j = int(np.argmin(np.abs(t - tt)))
        p = enu[j, :2]
        iv = imu_dir(tt) * arrow_len
        ax.arrow(p[0], p[1], iv[0], iv[1], color="tab:blue", width=0.4,
                 head_width=2.0, length_includes_head=True, zorder=4,
                 label="IMU heading" if k == 0 else None)
        cv, spd = course_dir(tt)
        if spd >= min_speed:
            cvec = cv * arrow_len
            ax.arrow(p[0], p[1], cvec[0], cvec[1], color="tab:red", width=0.4,
                     head_width=2.0, length_includes_head=True, zorder=3,
                     label="GNSS course" if k == 0 else None)
        ax.annotate(f"{tt-t[0]:.0f}s", (p[0], p[1]), textcoords="offset points",
                    xytext=(3, 3), fontsize=7, color="0.3")

    ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
    ax.set_title(f"IMU heading (blue) vs GNSS course (red) every {interval:.0f}s\n"
                 f"{cfg['run'].get('name', 'run')} — arrows {arrow_len:.0f} m")
    ax.axis("equal"); ax.legend(loc="best")
    fig.tight_layout()

    out = Path(output) if output else Path("outputs/imu_vs_gnss_headings.png")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"[plot_headings] wrote {out}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--interval", type=float, default=20.0, help="seconds between arrow pairs")
    ap.add_argument("--arrow-len", type=float, default=10.0, help="arrow length in metres")
    ap.add_argument("--min-speed", type=float, default=0.3,
                    help="skip GNSS-course arrow below this speed (m/s)")
    ap.add_argument("--output", default=None)
    args = ap.parse_args()
    plot_headings(args.config, args.interval, args.arrow_len, args.min_speed, args.output)


if __name__ == "__main__":
    main()
