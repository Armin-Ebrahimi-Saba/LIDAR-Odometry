#!/usr/bin/env python3
"""Compute RMSE (Absolute Trajectory Error) between a GLIM trajectory and
GNSS ground truth: error-over-time plot, top-down (local ENU) comparison
plot, and an interactive real-map overlay (OpenStreetMap + satellite tiles).

Note: evaluation is automatically scoped to only whatever portion of the
route GLIM actually processed. If GLIM stopped partway through the bag,
t_est simply won't contain later timestamps, so nothing beyond what GLIM
produced is compared against.

Usage: python3 compute_rmse.py <traj_lidar.txt> <gnss_csv> <output_prefix> [label]
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
import contextily as cx
from utils import (load_glim_traj, load_gnss, latlon_to_local_enu,
                    local_enu_to_latlon, compute_alignment)


def make_plots(t_matched, errors, src_aligned, dst, prefix, label):

    t_rel = t_matched - t_matched[0]
    rmse = np.sqrt(np.mean(errors ** 2))

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    axes[0].plot(t_rel, errors)
    axes[0].axhline(rmse, color='r', linestyle='--', label=f'RMSE = {rmse:.2f} m')
    axes[0].set_xlabel('Time [s]')
    axes[0].set_ylabel('Position error [m]')
    axes[0].set_title(f'Absolute Trajectory Error over time{" - " + label if label else ""}')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(dst[:, 0], dst[:, 1], 'g-', label='GNSS ground truth', linewidth=2)
    axes[1].plot(src_aligned[:, 0], src_aligned[:, 1], 'b--', label='GLIM (aligned)', linewidth=1.5)
    axes[1].set_xlabel('East [m]')
    axes[1].set_ylabel('North [m]')
    axes[1].set_title(f'Trajectory comparison (top-down){" - " + label if label else ""}')
    axes[1].legend()
    axes[1].axis('equal')
    axes[1].grid(True)

    plt.tight_layout()
    out = f"{prefix}_error_plot.png"
    plt.savefig(out, dpi=150)
    print(f"Saved: {out}")


def make_map_plot(xyz_est_aligned, lat0, lon0, alt0,
                   t_gt_matched, lats_gt_matched, lons_gt_matched, prefix, label, interval=100.0,
                   satellite=True):
    """Overlay both trajectories on a real map background, saved as a static PNG."""

    lats_est, lons_est, _ = local_enu_to_latlon(xyz_est_aligned, lat0, lon0, alt0)

    def to_web_mercator(lat, lon):
        k = 20037508.34 / 180.0
        x = lon * k
        y = np.log(np.tan((90 + lat) * np.pi / 360.0)) / (np.pi / 180.0) * k
        return x, y

    x_gt, y_gt = to_web_mercator(lats_gt_matched, lons_gt_matched)
    x_est, y_est = to_web_mercator(lats_est, lons_est)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(x_gt, y_gt, '-', color='lime', linewidth=3, label='GNSS ground truth',
            zorder=3, path_effects=None)
    ax.plot(x_est, y_est, '--', color='cyan', linewidth=2.5, label='GLIM (aligned)',
            zorder=3)

    # Time markers every `interval` seconds, on the GNSS track
    t0 = t_gt_matched[0]
    next_marker = 0.0
    for i, t in enumerate(t_gt_matched):
        if t - t0 >= next_marker:
            ax.scatter(x_gt[i], y_gt[i], color='yellow', edgecolor='black',
                       s=60, zorder=4)
            ax.annotate(f"{int(next_marker)}s", (x_gt[i], y_gt[i]),
                        textcoords="offset points", xytext=(6, 6), fontsize=9,
                        color='white', weight='bold',
                        path_effects=[__import__('matplotlib.patheffects', fromlist=['withStroke']).withStroke(linewidth=2, foreground='black')])
            next_marker += interval

    ax.scatter(x_gt[0], y_gt[0], color='lime', edgecolor='black', s=150,
               marker='*', zorder=5, label='start')
    ax.scatter(x_gt[-1], y_gt[-1], color='red', edgecolor='black', s=150,
               marker='X', zorder=5, label='end')

    ax.set_xlim(min(x_gt.min(), x_est.min()) - 30, max(x_gt.max(), x_est.max()) + 30)
    ax.set_ylim(min(y_gt.min(), y_est.min()) - 30, max(y_gt.max(), y_est.max()) + 30)

    source = cx.providers.Esri.WorldImagery if satellite else cx.providers.OpenStreetMap.Mapnik
    cx.add_basemap(ax, crs="EPSG:3857", source=source, zoom=18)

    ax.set_axis_off()
    ax.legend(loc='upper right', fontsize=11, framealpha=0.9)
    ax.set_title(f'Trajectory on map{" - " + label if label else ""}', fontsize=13)

    plt.tight_layout()
    out = f"{prefix}_satellite.png"
    plt.savefig(out, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"Saved: {out}")


def main():
    if len(sys.argv) < 4:
        print("Usage: compute_rmse.py <traj_lidar.txt> <gnss_csv> <output_prefix> [label]")
        sys.exit(1)

    traj_path, gnss_path, prefix = sys.argv[1], sys.argv[2], sys.argv[3]
    label = sys.argv[4] if len(sys.argv) > 4 else ""

    t_est, xyz_est = load_glim_traj(traj_path)
    t_gt, lats, lons, alts = load_gnss(gnss_path)

    print(f"GLIM trajectory: {len(t_est)} poses, {t_est[0]:.3f} -> {t_est[-1]:.3f} "
          f"({t_est[-1]-t_est[0]:.1f}s processed)")
    print(f"GNSS ground truth: {len(t_gt)} fixes, {t_gt[0]:.3f} -> {t_gt[-1]:.3f} "
          f"({t_gt[-1]-t_gt[0]:.1f}s)")

    lat0, lon0, alt0 = lats[0], lons[0], alts[0]
    xyz_gt = latlon_to_local_enu(lats, lons, alts, lat0, lon0, alt0)

    R, t_vec, matches = compute_alignment(t_est, xyz_est, t_gt, xyz_gt)
    print(f"Matched {len(matches)} / {len(t_est)} GLIM poses (within 0.1s)")

    idx_est = [m[0] for m in matches]
    idx_gt = [m[1] for m in matches]
    src = xyz_est[idx_est]
    dst = xyz_gt[idx_gt]
    t_matched = t_est[idx_est]

    src_aligned = (R @ src.T).T + t_vec
    errors = np.linalg.norm(src_aligned - dst, axis=1)

    t_gt_matched = t_gt[idx_gt]
    lats_gt_matched = lats[idx_gt]
    lons_gt_matched = lons[idx_gt]

    rmse = np.sqrt(np.mean(errors ** 2))
    mean_err, median_err, max_err, std_err = errors.mean(), np.median(errors), errors.max(), errors.std()

    print()
    print(f"=== Absolute Trajectory Error{' - ' + label if label else ''} ===")
    print(f"RMSE:   {rmse:.3f} m")
    print(f"Mean:   {mean_err:.3f} m")
    print(f"Median: {median_err:.3f} m")
    print(f"Std:    {std_err:.3f} m")
    print(f"Max:    {max_err:.3f} m")

    np.savetxt(f"{prefix}_errors.csv", np.column_stack([t_matched, errors]),
               header="timestamp,error_m", delimiter=",", comments="")
    np.savetxt(f"{prefix}_aligned_trajectory.csv",
               np.column_stack([t_matched, src_aligned, dst]),
               header="timestamp,est_x,est_y,est_z,gt_x,gt_y,gt_z", delimiter=",", comments="")
    print(f"\nSaved: {prefix}_errors.csv, {prefix}_aligned_trajectory.csv")

    make_plots(t_matched, errors, src_aligned, dst, prefix, label)

    # Full (not just matched) GLIM trajectory, aligned, for a smoother map overlay
    xyz_full_aligned = (R @ xyz_est.T).T + t_vec
    make_map_plot(xyz_full_aligned, lat0, lon0, alt0, t_gt_matched, lats_gt_matched, lons_gt_matched, prefix, label)


if __name__ == "__main__":
    main()
