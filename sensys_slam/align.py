"""Tie the local SLAM trajectory to the GNSS ground truth.

The SLAM trajectory lives in a world frame whose origin is the first GNSS
ground-truth point (seeded into the odometry), but whose *orientation* is still
arbitrary (the LiDAR's initial heading). This stage finds the rigid SE(3)
transform that brings it onto the GNSS ENU frame.

Procedure:
  1. Convert the ground truth to ENU metres around its own first sample.
  2. Time-match SLAM poses to ground-truth samples (nearest-neighbour within a
     tolerance).
  3. Estimate R, t with a global least-squares (Umeyama) best fit, optionally
     weighted by the GNSS solution's own uncertainty (eph).
  4. Apply R, t to the entire trajectory, then convert back to lat/lon.
"""
import numpy as np
import pandas as pd

from .geo import geodetic_to_enu, enu_to_geodetic


def _normalize_weights(n: int, weights):
    """Return a length-`n` non-negative weight vector (uniform if `weights` is
    None). Lets the alignment be optionally eph-weighted."""
    if weights is None:
        return np.ones(n)
    w = np.asarray(weights, float).ravel()
    if w.shape[0] != n:
        raise ValueError(f"weights length {w.shape[0]} != {n} matched pairs")
    if np.any(w < 0) or not np.all(np.isfinite(w)) or w.sum() <= 0:
        raise ValueError("weights must be finite, non-negative, and not all zero")
    return w


def umeyama_alignment(src: np.ndarray, dst: np.ndarray, weights=None):
    """Global least-squares R, t (scale fixed to 1) with dst ~= (R @ src.T).T + t.

    With `weights` (per-pair, e.g. 1/eph^2) the centroids and cross-covariance
    are weighted, so uncertain ground-truth samples pull the fit less."""
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError("umeyama_alignment needs matching (N>=3, 3) arrays")
    w = _normalize_weights(src.shape[0], weights)
    wn = w / w.sum()
    mu_src = (wn[:, None] * src).sum(0)
    mu_dst = (wn[:, None] * dst).sum(0)
    src_c, dst_c = src - mu_src, dst - mu_dst
    cov = (dst_c * wn[:, None]).T @ src_c
    U, _, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1.0
    R = U @ S @ Vt
    return R, mu_dst - R @ mu_src


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


def match_weights(gt_df: pd.DataFrame, ref_idx, cfg: dict):
    """Per-pair inverse-variance weights for matched ground-truth samples.

    Uses PX4's own horizontal position uncertainty `eph` (metres, 1-sigma):
    weight = 1 / max(eph, eph_floor)^2, so uncertain GT (e.g. the several-metre
    wander during GNSS initialisation, where eph is worst) is down-weighted in
    the alignment fit and RMSE. The floor stops a single over-confident sample
    from dominating. Returns None (=> uniform weighting) when eph weighting is
    disabled or the `eph` column is absent.
    """
    acfg = cfg.get("alignment", {})
    if not acfg.get("eph_weighting", True) or "eph" not in gt_df.columns:
        return None
    floor = float(acfg.get("eph_floor_m", 0.3))
    eph = np.asarray(gt_df["eph"].values, float)[np.asarray(ref_idx)]
    bad = ~np.isfinite(eph) | (eph <= 0)
    if bad.any():                       # fall back to the median for missing eph
        med = np.nanmedian(np.where(bad, np.nan, eph))
        eph = np.where(bad, med if np.isfinite(med) else floor, eph)
    return 1.0 / np.maximum(eph, floor) ** 2


def align_and_georeference(poses_df: pd.DataFrame, gt_df: pd.DataFrame, cfg: dict,
                           ref_origin=None):
    """Returns (traj_latlon_df, ref_origin, fit_rmse_m, (R, t)).

    Aligns the odometry onto the GNSS ENU frame with a global least-squares
    (Umeyama) rigid fit, optionally eph-weighted.

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
    w = match_weights(gt_df, r_idx, cfg)
    R, t = umeyama_alignment(src, dst, w)

    fit_pred = (R @ src.T).T + t
    sq = np.sum((fit_pred - dst) ** 2, axis=1)
    # Report the eph-weighted fit RMSE when weighting is active (matches the
    # quantity actually minimised); falls back to the plain RMSE otherwise.
    fit_rmse_m = float(np.sqrt(np.average(sq, weights=w) if w is not None
                               else np.mean(sq)))

    all_xyz = poses_df[["x", "y", "z"]].values
    aligned = (R @ all_xyz.T).T + t
    lat, lon, alt = enu_to_geodetic(aligned, lat0, lon0, alt0)

    traj = pd.DataFrame({
        "timestamp": poses_df["timestamp"].values,
        "lat": lat, "lon": lon, "alt": alt,
        "x_enu": aligned[:, 0], "y_enu": aligned[:, 1], "z_enu": aligned[:, 2],
    })
    return traj, (lat0, lon0, alt0), fit_rmse_m, (R, t)
