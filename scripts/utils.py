#!/usr/bin/env python3
import numpy as np
import csv

EARTH_RADIUS = 6371000.0 # in [m]

def load_glim_traj(path):
    """GLIM TUM format: t x y z qx qy qz qw"""
    data = np.loadtxt(path)

    return data[:, 0], data[:, 1:4]



def load_gnss(path):
    """Load provided xtrack_global_position.csv"""

    times, lats, lons, alts = [], [], [], []

    with open(path) as f:

        reader = csv.DictReader(f)

        for row in reader:

            if row.get("lat_lon_valid", "1") == "0":

                continue

            times.append(float(row["timestamp"]))

            lats.append(float(row["lat"]))

            lons.append(float(row["lon"]))

            alts.append(float(row["alt"]))

    return np.array(times), np.array(lats), np.array(lons), np.array(alts)



def latlon_to_local_enu(lats, lons, alts, lat0, lon0, alt0):

    """Equirectangular approximation for city-scale routes."""

    lat0_rad = np.radians(lat0)

    east = np.radians(lons - lon0) * EARTH_RADIUS * np.cos(lat0_rad)

    north = np.radians(lats - lat0) * EARTH_RADIUS

    up = alts - alt0

    return np.column_stack([east, north, up])



def local_enu_to_latlon(enu, lat0, lon0, alt0):

    lat0_rad = np.radians(lat0)

    lats = lat0 + np.degrees(enu[:, 1] / EARTH_RADIUS)

    lons = lon0 + np.degrees(enu[:, 0] / (EARTH_RADIUS * np.cos(lat0_rad)))

    alts = alt0 + enu[:, 2]

    return lats, lons, alts



def associate(t_est, t_gt, max_dt=0.1):

    """Nearest-neighbor timestamp matching (both arrays must be sorted)."""

    matches = []

    j = 0

    for i, t in enumerate(t_est):

        while j + 1 < len(t_gt) and abs(t_gt[j + 1] - t) <= abs(t_gt[j] - t):

            j += 1

        if abs(t_gt[j] - t) <= max_dt:

            matches.append((i, j))

    return matches



def umeyama_alignment(src, dst):

    """Horn's method or Umeyama without scaling: find R, t minimizing

    ||dst - (R @ src + t)||^2. src, dst: Nx3 arrays.


    This resolves the unknown rigid transform (rotation + translation)

    between GLIM's arbitrary local start-frame and the GNSS local-ENU frame.

    It does NOT fuse GNSS into the estimate, rather only anchors an otherwise

    frame-less trajectory into world coordinates, the same way any SLAM

    evaluation (e.g. the 'evo' toolkit's ATE metric, used in the GLIM paper

    itself) aligns an estimated trajectory to ground truth before comparing.

    """

    src_mean, dst_mean = src.mean(axis=0), dst.mean(axis=0)

    src_c, dst_c = src - src_mean, dst - dst_mean

    H = src_c.T @ dst_c

    U, S, Vt = np.linalg.svd(H)

    d = np.sign(np.linalg.det(Vt.T @ U.T))

    R = Vt.T @ np.diag([1, 1, d]) @ U.T

    t = dst_mean - R @ src_mean

    return R, t



def compute_alignment(t_est, xyz_est, t_gt, xyz_gt, max_dt=0.1):

    """Convenience wrapper: associate + align. Returns R, t, and the match list."""

    matches = associate(t_est, t_gt, max_dt=max_dt)

    if len(matches) < 10:

        raise RuntimeError(f"Only {len(matches)} timestamp matches found."

                            f"Please check that the two files overlap in time")

    idx_est = [m[0] for m in matches]

    idx_gt = [m[1] for m in matches]

    R, t = umeyama_alignment(xyz_est[idx_est], xyz_gt[idx_gt])

    return R, t, matches

