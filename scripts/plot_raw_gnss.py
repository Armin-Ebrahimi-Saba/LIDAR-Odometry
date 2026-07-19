
#!/usr/bin/env python3
"""Plot the raw GNSS ground-truth trajectory alone (local ENU), with time
markers every N seconds -- to visually check for the 'large GNSS track jump'
mentioned in the course slides, independent of any GLIM comparison.

Usage: python3 plot_raw_gnss.py <gnss_csv> [output.png] [marker_interval_s]
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
from utils import load_gnss, latlon_to_local_enu


def main():
    gnss_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else "raw_gnss_trajectory.png"
    interval = float(sys.argv[3]) if len(sys.argv) > 3 else 100.0

    t_gt, lats, lons, alts = load_gnss(gnss_path)
    lat0, lon0, alt0 = lats[0], lons[0], alts[0]
    xyz = latlon_to_local_enu(lats, lons, alts, lat0, lon0, alt0)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(xyz[:, 0], xyz[:, 1], '-', color='green', linewidth=1, alpha=0.7)
    ax.scatter(xyz[0, 0], xyz[0, 1], color='black', s=80, zorder=5, label='start')
    ax.scatter(xyz[-1, 0], xyz[-1, 1], color='red', marker='x', s=100, zorder=5, label='end')

    t0 = t_gt[0]
    next_marker = 0.0
    for i, t in enumerate(t_gt):
        if t - t0 >= next_marker:
            ax.scatter(xyz[i, 0], xyz[i, 1], color='blue', s=30, zorder=4)
            ax.annotate(f"{int(next_marker)}s", (xyz[i, 0], xyz[i, 1]),
                        textcoords="offset points", xytext=(5, 5), fontsize=8)
            next_marker += interval

    ax.set_xlabel('East [m]')
    ax.set_ylabel('North [m]')
    ax.set_title(f'Raw GNSS trajectory (ground truth only), markers every {interval:.0f}s')
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"Saved: {out_path}")
    print(f"Total duration: {t_gt[-1]-t_gt[0]:.1f}s, {len(t_gt)} fixes")

    # Also flag any single-step jump that's physically implausible
    # (Xtrack is a ground vehicle -- even at high speed, >10m between
    # consecutive ~0.1-0.2s-spaced fixes would be a red flag)
    dists = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)
    dts = np.diff(t_gt)
    speeds = dists / np.where(dts > 0, dts, np.nan)
    jump_idx = np.argsort(dists)[::-1][:10]
    print("\nLargest single-step position jumps:")
    for i in jump_idx:
        print(f"  t={t_gt[i]-t0:.1f}s  dist={dists[i]:.1f}m  dt={dts[i]:.2f}s  "
              f"implied_speed={speeds[i]:.1f}m/s")


if __name__ == "__main__":
    main()
