#!/usr/bin/env python3

"""Export GLIM's estimated trajectory as 2D LatLon position + NED velocity as .csv file.


GLIM's raw output is in an arbitrary local frame (origin = start pose,

orientation from gravity alignment only with no true heading reference). To

express it as LatLon, it must be aligned to the GNSS ground truth's local-ENU frame

via a rigid SE3 transform. Velocity is then differentiated from the aligned positions

and rotated into NED frame.

Usage: python3 export_trajectory.py <traj_lidar.txt> <gnss_csv> [output.csv]
"""

import sys
import numpy as np
import csv
from utils import (load_glim_traj, load_gnss, latlon_to_local_enu, local_enu_to_latlon, compute_alignment)


def main():
    if len(sys.argv) < 3:

        print("Usage: export_trajectory.py <traj_lidar.txt> <gnss_csv> [output.csv]")

        sys.exit(1)


    traj_path, gnss_path = sys.argv[1], sys.argv[2]

    out_path = sys.argv[3] if len(sys.argv) > 3 else "trajectory_latlon_ned.csv"


    t_est, xyz_est = load_glim_traj(traj_path)

    t_gt, lats, lons, alts = load_gnss(gnss_path)

    offset = t_est[0] - t_gt[0]
    t_gt = t_gt + offset
    print(f"Applied GNSS offset: {offset:.3f}s")

    lat0, lon0, alt0 = lats[0], lons[0], alts[0]

    xyz_gt = latlon_to_local_enu(lats, lons, alts, lat0, lon0, alt0)


    R, t_vec, matches = compute_alignment(t_est, xyz_est, t_gt, xyz_gt)

    print(f"Aligned using {len(matches)} matched poses")


    # Apply alignment to the FULL GLIM trajectory (not just the matched subset)

    xyz_aligned = (R @ xyz_est.T).T + t_vec

    lats_est, lons_est, alts_est = local_enu_to_latlon(xyz_aligned, lat0, lon0, alt0)


    # Velocity via finite differences in the aligned ENU frame, then -> NED

    # (North = ENU_y, East = ENU_x, Down = -ENU_z)

    dt = np.gradient(t_est)

    dt[dt == 0] = np.nan

    v_north = np.gradient(xyz_aligned[:, 1]) / dt

    v_east = np.gradient(xyz_aligned[:, 0]) / dt

    v_down = -np.gradient(xyz_aligned[:, 2]) / dt


    with open(out_path, "w", newline="") as f:

        writer = csv.writer(f)

        writer.writerow(["timestamp", "lat_deg", "lon_deg", "alt_m",

                          "vel_N_mps", "vel_E_mps", "vel_D_mps"])

        for i in range(len(t_est)):

            writer.writerow([

                f"{t_est[i]:.6f}", f"{lats_est[i]:.9f}", f"{lons_est[i]:.9f}",

                f"{alts_est[i]:.3f}", f"{v_north[i]:.4f}", f"{v_east[i]:.4f}", f"{v_down[i]:.4f}",

            ])


    print(f"Wrote {len(t_est)} poses to {out_path}")

    print(f"Reference origin (GNSS first fix): lat={lat0:.6f}, lon={lon0:.6f}, alt={alt0:.2f}")



if __name__ == "__main__":

    main()

