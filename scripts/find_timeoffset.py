#!/usr/bin/env python3
"""Scan for a constant time offset between GLIM and GNSS timestamps that
minimizes RMSE. If a clear minimum exists away from offset=0, the two
sensors' clocks are not synchronized and every prior RMSE was measuring
alignment against a temporally-shifted ground truth.

Usage: python3 find_time_offset.py <traj_lidar.txt> <gnss_csv>
"""
import sys
import numpy as np
from utils import load_glim_traj, load_gnss, latlon_to_local_enu, compute_alignment


def rmse_at_offset(t_est, xyz_est, t_gt, xyz_gt, offset):
    try:
        R, t_vec, matches = compute_alignment(t_est + offset, xyz_est, t_gt, xyz_gt, max_dt=0.1)
    except RuntimeError:
        return None, 0
    idx_est = [m[0] for m in matches]
    idx_gt = [m[1] for m in matches]
    src_aligned = (R @ xyz_est[idx_est].T).T + t_vec
    errors = np.linalg.norm(src_aligned - xyz_gt[idx_gt], axis=1)
    return np.sqrt(np.mean(errors ** 2)), len(matches)


def main():
    traj_path, gnss_path = sys.argv[1], sys.argv[2]
    t_est, xyz_est = load_glim_traj(traj_path)
    t_gt, lats, lons, alts = load_gnss(gnss_path)
    lat0, lon0, alt0 = lats[0], lons[0], alts[0]
    xyz_gt = latlon_to_local_enu(lats, lons, alts, lat0, lon0, alt0)

    print(f"{'offset[s]':>10} {'RMSE[m]':>10} {'matches':>8}")
    results = []
    for offset in np.arange(-20.0, 20.01, 1.0):
        rmse, n = rmse_at_offset(t_est, xyz_est, t_gt, xyz_gt, offset)
        if rmse is not None:
            results.append((offset, rmse))
            print(f"{offset:10.1f} {rmse:10.3f} {n:8d}")

    best_offset, best_rmse = min(results, key=lambda r: r[1])
    print(f"\nBest offset: {best_offset:.1f}s -> RMSE {best_rmse:.3f}m")
    print("If this is much lower than the offset=0 RMSE and the minimum is")
    print("clearly localized (not flat/noisy), the two clocks are likely offset")
    print("by roughly this amount -- refine with a finer scan around it.")


if __name__ == "__main__":
    main()
