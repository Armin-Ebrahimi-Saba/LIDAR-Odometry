"""Tie the local SLAM trajectory to the GNSS ground truth.

The SLAM trajectory lives in a world frame whose origin is the first GNSS
ground-truth point (seeded into the odometry), but whose *orientation* is still
arbitrary (the LiDAR's initial heading). This stage finds the rigid SE(3)
transform that brings it onto the GNSS ENU frame.

Procedure:
  1. Convert the ground truth to ENU metres around its own first sample.
  2. Time-match SLAM poses to ground-truth samples (nearest-neighbour within a
     tolerance).
  3. Estimate R, t with the *first* matched pair pinned exactly -- the SLAM
     start is anchored onto the GNSS start and the rotation about that anchor
     is fit to the rest -- so the error curve starts at 0 and grows as drift
     from a known origin. (A global Umeyama best fit is also available.)
  4. Apply R, t to the entire trajectory, then convert back to lat/lon.
"""
import numpy as np
import pandas as pd

from .geo import geodetic_to_enu, enu_to_geodetic


def umeyama_alignment(src: np.ndarray, dst: np.ndarray):
    """Global least-squares R, t (scale fixed to 1) with dst ~= (R @ src.T).T + t."""
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError("umeyama_alignment needs matching (N>=3, 3) arrays")
    mu_src, mu_dst = src.mean(0), dst.mean(0)
    src_c, dst_c = src - mu_src, dst - mu_dst
    cov = dst_c.T @ src_c / src.shape[0]
    U, _, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1.0
    R = U @ S @ Vt
    return R, mu_dst - R @ mu_src


def anchored_alignment(src: np.ndarray, dst: np.ndarray):
    """Rigid R, t with the FIRST pair pinned exactly (`R @ src[0] + t == dst[0]`)
    and the rotation about that anchor fit to the rest (orthogonal Procrustes).
    Error at t=0 is zero; RMSE is higher than Umeyama but reads as drift from a
    known origin."""
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError("anchored_alignment needs matching (N>=3, 3) arrays")
    src0, dst0 = src[0], dst[0]
    src_c, dst_c = src - src0, dst - dst0
    cov = dst_c.T @ src_c
    U, _, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1.0
    R = U @ S @ Vt
    return R, dst0 - R @ src0


def nearest_time_match(query_times, ref_times, max_diff):
    """For each query time, index of the nearest ref time; keep pairs within
    `max_diff`. Returns (query_idx, ref_idx)."""
    ref_times = np.asarray(ref_times)
    order = np.argsort(ref_times)
    ref_sorted = ref_times[order]
    pos = np.clip(np.searchsorted(ref_sorted, query_times), 1, len(ref_sorted) - 1)
    left, right = pos - 1, pos
    use_left = np.abs(query_times - ref_sorted[left]) <= np.abs(query_times - ref_sorted[right])
    nearest_sorted = np.where(use_left, left, right)
    nearest_diff = np.where(use_left,
                            np.abs(query_times - ref_sorted[left]),
                            np.abs(query_times - ref_sorted[right]))
    nearest_ref = order[nearest_sorted]
    valid = nearest_diff <= max_diff
    return np.nonzero(valid)[0], nearest_ref[valid]


def align_and_georeference(poses_df: pd.DataFrame, gt_df: pd.DataFrame, cfg: dict,
                           ref_origin=None):
    """Returns (traj_latlon_df, ref_origin, fit_rmse_m, (R, t)).

    If `ref_origin` (lat0, lon0, alt0) is given it is used as the ENU tangent
    point (so it matches the origin the odometry was seeded with); otherwise
    the ground truth's first sample is used.
    """
    if ref_origin is None:
        ref_origin = (float(gt_df["lat"].iloc[0]), float(gt_df["lon"].iloc[0]),
                      float(gt_df["alt"].iloc[0]))
    lat0, lon0, alt0 = ref_origin

    gt_enu = geodetic_to_enu(gt_df["lat"].values, gt_df["lon"].values,
                             gt_df["alt"].values, lat0, lon0, alt0)

    max_diff = cfg.get("alignment", {}).get("max_time_diff_s", 0.15)
    q_idx, r_idx = nearest_time_match(poses_df["timestamp"].values,
                                      gt_df["timestamp"].values, max_diff)
    if len(q_idx) < 10:
        raise RuntimeError(
            f"Only {len(q_idx)} timestamp matches between SLAM poses and ground "
            f"truth within {max_diff}s. Check time windows / shared epoch.")

    src = poses_df[["x", "y", "z"]].values[q_idx]
    dst = gt_enu[r_idx]
    method = cfg.get("alignment", {}).get("method", "anchored")
    R, t = (anchored_alignment(src, dst) if method == "anchored"
            else umeyama_alignment(src, dst))

    fit_pred = (R @ src.T).T + t
    fit_rmse_m = float(np.sqrt(np.mean(np.sum((fit_pred - dst) ** 2, axis=1))))

    all_xyz = poses_df[["x", "y", "z"]].values
    aligned = (R @ all_xyz.T).T + t
    lat, lon, alt = enu_to_geodetic(aligned, lat0, lon0, alt0)

    traj = pd.DataFrame({
        "timestamp": poses_df["timestamp"].values,
        "lat": lat, "lon": lon, "alt": alt,
        "x_enu": aligned[:, 0], "y_enu": aligned[:, 1], "z_enu": aligned[:, 2],
    })
    return traj, (lat0, lon0, alt0), fit_rmse_m, (R, t)
