#!/usr/bin/env python3
"""Scan for a constant time offset between GLIM and GNSS timestamps that
minimizes RMSE. Uses a coarse-to-fine search approach.

Usage:
  python3 scripts/find_timeoffset.py <traj_lidar.txt> <gnss_csv> [--min 150.0] [--max 190.0] [--step 1.0]
"""
import sys
import argparse
import numpy as np
from utils import load_glim_traj, load_gnss, latlon_to_local_enu, compute_alignment


def rmse_at_offset(t_est, xyz_est, t_gt, xyz_gt, offset, max_dt=0.1):
    try:
        R, t_vec, matches = compute_alignment(t_est + offset, xyz_est, t_gt, xyz_gt, max_dt=max_dt)
    except RuntimeError:
        return None, 0
    if len(matches) < 5:
        return None, len(matches)
    idx_est = [m[0] for m in matches]
    idx_gt = [m[1] for m in matches]
    src_aligned = (R @ xyz_est[idx_est].T).T + t_vec
    errors = np.linalg.norm(src_aligned - xyz_gt[idx_gt], axis=1)
    return np.sqrt(np.mean(errors ** 2)), len(matches)


def scan_range(t_est, xyz_est, t_gt, xyz_gt, start, stop, step):
    results = []
    print(f"\nScanning offsets from {start:.2f}s to {stop:.2f}s (step {step:.2f}s)...")
    print(f"{'offset[s]':>10} {'RMSE[m]':>10} {'matches':>8}")

    for offset in np.arange(start, stop + step * 0.5, step):
        rmse, n = rmse_at_offset(t_est, xyz_est, t_gt, xyz_gt, offset)
        if rmse is not None and n > 0:
            results.append((offset, rmse, n))
            print(f"{offset:10.2f} {rmse:10.3f} {n:8d}")
        else:
            print(f"{offset:10.2f} {'N/A':>10} {n:8d}")

    if not results:
        return None, None
    best = min(results, key=lambda r: r[1])
    return best[0], best[1]


def main():
    parser = argparse.ArgumentParser(description="Find time offset between GLIM and GNSS ground truth.")
    parser.add_argument("traj_path", help="Path to GLIM trajectory (e.g. traj_lidar.txt)")
    parser.add_argument("gnss_path", help="Path to GNSS CSV file")
    parser.add_argument("--min", type=float, default=150.0, help="Min offset search range (default: 150.0)")
    parser.add_argument("--max", type=float, default=190.0, help="Max offset search range (default: 190.0)")
    parser.add_argument("--step", type=float, default=1.0, help="Coarse search step size (default: 1.0)")
    args = parser.parse_args()

    t_est, xyz_est = load_glim_traj(args.traj_path)
    t_gt, lats, lons, alts = load_gnss(args.gnss_path)
    lat0, lon0, alt0 = lats[0], lons[0], alts[0]
    xyz_gt = latlon_to_local_enu(lats, lons, alts, lat0, lon0, alt0)

    # 1. Coarse Sweep
    best_coarse, rmse_coarse = scan_range(t_est, xyz_est, t_gt, xyz_gt, args.min, args.max, args.step)
    if best_coarse is None:
        print("\n[Error] No point matches found in search range. Try expanding --min and --max.")
        return

    print(f"\n--> Coarse best offset: {best_coarse:.2f}s (RMSE: {rmse_coarse:.3f}m)")

    # 2. Fine Sweep (+- 2.0s around coarse best, step 0.05s)
    fine_min = best_coarse - 2.0
    fine_max = best_coarse + 2.0
    best_fine, rmse_fine = scan_range(t_est, xyz_est, t_gt, xyz_gt, fine_min, fine_max, step=0.05)

    print("\n==============================================")
    print(f"FINAL OPTIMAL OFFSET: {best_fine:.3f} s")
    print(f"MINIMUM TRAJECTORY RMSE: {rmse_fine:.3f} m")
    print("==============================================")


if __name__ == "__main__":
    main()
