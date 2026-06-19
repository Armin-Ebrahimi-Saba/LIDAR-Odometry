"""Tie the local SLAM trajectory to the GNSS ground truth.

The SLAM trajectory lives in an arbitrary local Cartesian frame (origin =
first scan pose, axes = whatever the LiDAR's initial orientation happened to
be). The ground truth lives in WGS84 lat/lon/alt. To compare or merge them
we need a rigid SE(3) transform (rotation + translation, scale fixed to 1
since both are already metric) that maps the SLAM frame onto a local ENU
frame anchored at the ground truth.

Procedure:
  1. Convert the ground truth to ENU meters around its own first sample.
  2. Time-match SLAM poses to ground-truth samples (nearest-neighbor within
     a tolerance).
  3. Estimate R, t via Umeyama/Kabsch (no scaling) from the matched pairs.
  4. Apply R, t to the *entire* SLAM trajectory (not just the matched
     subset), then convert the result back to lat/lon.
"""
import numpy as np
import pandas as pd

from .geo import geodetic_to_enu, enu_to_geodetic


def umeyama_alignment(src: np.ndarray, dst: np.ndarray):
    """Estimate rotation R and translation t such that dst ~= (R @ src.T).T + t.

    Scale is fixed to 1 because both point sets are already in meters.
    src, dst: (N, 3) arrays of corresponding points, N >= 3.
    Returns: R (3,3), t (3,)
    """
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError("umeyama_alignment needs matching (N>=3, 3) arrays")

    mu_src, mu_dst = src.mean(axis=0), dst.mean(axis=0)
    src_c, dst_c = src - mu_src, dst - mu_dst

    cov = dst_c.T @ src_c / src.shape[0]
    U, _, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1.0
    R = U @ S @ Vt
    t = mu_dst - R @ mu_src
    return R, t


def nearest_time_match(query_times: np.ndarray, ref_times: np.ndarray, max_diff: float):
    """For each value in `query_times`, find the index of the nearest value
    in `ref_times`. Only pairs within `max_diff` seconds are kept.

    Returns (query_idx, ref_idx): index arrays into the original arrays for
    the kept matches, same length, ordered by query_idx.
    """
    ref_times = np.asarray(ref_times)
    order = np.argsort(ref_times)
    ref_sorted = ref_times[order]

    pos = np.searchsorted(ref_sorted, query_times)
    pos = np.clip(pos, 1, len(ref_sorted) - 1)
    left, right = pos - 1, pos
    left_diff = np.abs(query_times - ref_sorted[left])
    right_diff = np.abs(query_times - ref_sorted[right])
    use_left = left_diff <= right_diff
    nearest_sorted_idx = np.where(use_left, left, right)
    nearest_diff = np.where(use_left, left_diff, right_diff)

    nearest_ref_idx = order[nearest_sorted_idx]
    valid = nearest_diff <= max_diff
    query_idx = np.nonzero(valid)[0]
    ref_idx = nearest_ref_idx[valid]
    return query_idx, ref_idx


def align_and_georeference(poses_df: pd.DataFrame, gt_df: pd.DataFrame, cfg: dict):
    """
    poses_df: columns timestamp, x, y, z, qx, qy, qz, qw (local SLAM frame)
    gt_df:    columns timestamp, lat, lon, alt (WGS84, already cropped/filtered)

    Returns:
        traj_latlon_df : timestamp, lat, lon, alt, x_enu, y_enu, z_enu for
                          ALL SLAM poses, re-expressed in the ground truth's
                          ENU/geodetic frame.
        ref_origin      : (lat0, lon0, alt0) tangent point used for ENU.
        fit_rmse_m      : RMSE of the alignment fit itself, on the matched
                          calibration points only. This is a sanity check on
                          how well the rigid-transform assumption holds --
                          NOT the trajectory's overall accuracy (compute that
                          separately in sensys_slam.evaluate against the
                          full, independent ground-truth series).
        (R, t)          : the estimated rotation matrix and translation.
    """
    lat0 = float(gt_df["lat"].iloc[0])
    lon0 = float(gt_df["lon"].iloc[0])
    alt0 = float(gt_df["alt"].iloc[0])

    gt_enu = geodetic_to_enu(
        gt_df["lat"].values, gt_df["lon"].values, gt_df["alt"].values, lat0, lon0, alt0
    )

    max_diff = cfg.get("alignment", {}).get("max_time_diff_s", 0.15)
    q_idx, r_idx = nearest_time_match(poses_df["timestamp"].values, gt_df["timestamp"].values, max_diff)
    if len(q_idx) < 10:
        raise RuntimeError(
            f"Only {len(q_idx)} timestamp matches found between SLAM poses and "
            f"ground truth within {max_diff}s. Check that poses_local.csv and "
            f"the ground-truth CSV cover overlapping time windows and share "
            f"the same epoch (Unix seconds)."
        )

    src = poses_df[["x", "y", "z"]].values[q_idx]
    dst = gt_enu[r_idx]
    R, t = umeyama_alignment(src, dst)

    fit_pred = (R @ src.T).T + t
    fit_rmse_m = float(np.sqrt(np.mean(np.sum((fit_pred - dst) ** 2, axis=1))))

    all_xyz = poses_df[["x", "y", "z"]].values
    aligned_enu = (R @ all_xyz.T).T + t
    lat, lon, alt = enu_to_geodetic(aligned_enu, lat0, lon0, alt0)

    traj_latlon_df = pd.DataFrame(
        {
            "timestamp": poses_df["timestamp"].values,
            "lat": lat, "lon": lon, "alt": alt,
            "x_enu": aligned_enu[:, 0], "y_enu": aligned_enu[:, 1], "z_enu": aligned_enu[:, 2],
        }
    )
    return traj_latlon_df, (lat0, lon0, alt0), fit_rmse_m, (R, t)
