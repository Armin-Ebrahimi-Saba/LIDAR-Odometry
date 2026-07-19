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


def rotation_about_anchor(src: np.ndarray, dst: np.ndarray, weights=None):
    """Best-fit rotation only (no translation), with dst ~= (R @ src.T).T.

    Same SVD as `umeyama_alignment` but WITHOUT removing the centroid, so the fit
    is anchored at the origin: pass start-relative vectors (src/dst already have
    their shared start subtracted) and R is the rotation about that fixed start
    that best matches the rest. Optionally eph-weighted."""
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError("rotation_about_anchor needs matching (N>=3, 3) arrays")
    w = _normalize_weights(src.shape[0], weights)
    cov = (dst * w[:, None]).T @ src
    U, _, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1.0
    return U @ S @ Vt


def _ned_to_enu() -> np.ndarray:
    """Rotation taking a vector from NED (north, east, down) to ENU (east, north,
    up): swap N/E and flip D. Constant; the PX4 attitude is body(FRD)->NED, and
    the SLAM/GNSS work in ENU, so this bridges the two."""
    return np.array([[0.0, 1.0, 0.0],
                     [1.0, 0.0, 0.0],
                     [0.0, 0.0, -1.0]])


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


def remap_times_proportional(pose_times, gt_times):
    """Linearly map pose timestamps onto the GT time span so the first/last scan
    align with the first/last GT sample.

    For clocks that do NOT share an epoch but whose runs correspond end-to-end
    (first scan <-> first GT row, last scan <-> last GT row): scan fraction 0 maps
    to the GT start and fraction 1 to the GT end, so the middle is matched by
    proportion of the run rather than by absolute time. Uses the first/last *rows*
    (run endpoints), not min/max, since the clocks are unrelated."""
    pose_times = np.asarray(pose_times, float)
    p0, p1 = pose_times[0], pose_times[-1]
    g0, g1 = float(gt_times[0]), float(gt_times[-1])
    if p1 <= p0:
        return pose_times.copy()
    return g0 + (pose_times - p0) / (p1 - p0) * (g1 - g0)


def _cumulative_fraction(xy: np.ndarray) -> np.ndarray:
    """Cumulative path length along `xy`, normalised to [0, 1] (0 at the first
    point, 1 at the last). Monotonic non-decreasing; a stationary stretch adds
    ~0, so it collapses to a single fraction."""
    seg = np.linalg.norm(np.diff(np.asarray(xy, float), axis=0), axis=1)
    d = np.concatenate([[0.0], np.cumsum(seg)])
    return d / d[-1] if d[-1] > 0 else d


def _pose_positions(df: pd.DataFrame) -> np.ndarray:
    """Horizontal (x, y) of a poses frame or a trajectory frame."""
    if {"x", "y"}.issubset(df.columns):
        return df[["x", "y"]].values
    return df[["x_enu", "y_enu"]].values


def match_poses_to_gt(poses_df: pd.DataFrame, gt_df: pd.DataFrame, cfg: dict):
    """Time-match SLAM poses to GT samples per `alignment.time_match`:

      * "absolute" (default) -- nearest shared-clock timestamp within
        `alignment.max_time_diff_s`;
      * "proportional" -- the clocks don't share an epoch, so remap scan times
        onto the GT span (endpoints correspond) and take the nearest, with no
        absolute tolerance (every scan matches some GT sample);
      * "arclength" -- also endpoint-corresponding, but match by fraction of
        distance travelled instead of time. When the dwell/speed profiles of the
        two streams differ (e.g. the LiDAR sits stationary at the start far longer
        than the GT window does), time fraction pairs a parked scan with a GT
        point that has already moved off; path fraction does not, since a
        stationary stretch is ~0 arc length on both sides.

    Returns (pose_idx, gt_idx). Works on either a poses or trajectory frame (both
    carry a `timestamp` column; positions come from x/y or x_enu/y_enu)."""
    pt = poses_df["timestamp"].values
    gtt = gt_df["timestamp"].values
    acfg = cfg.get("alignment", {})
    mode = acfg.get("time_match", "absolute")
    if mode == "arclength":
        gt_xy = geodetic_to_enu(gt_df["lat"].values, gt_df["lon"].values,
                                np.zeros(len(gt_df)), float(gt_df["lat"].iloc[0]),
                                float(gt_df["lon"].iloc[0]), 0.0)[:, :2]
        pose_frac = _cumulative_fraction(_pose_positions(poses_df))
        gt_frac = _cumulative_fraction(gt_xy)
        return nearest_time_match(pose_frac, gt_frac, np.inf)
    if mode == "proportional":
        return nearest_time_match(remap_times_proportional(pt, gtt), gtt, np.inf)
    return nearest_time_match(pt, gtt, float(acfg.get("max_time_diff_s", 0.15)))


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


def attitude_anchored_alignment(poses_df: pd.DataFrame, gt_df: pd.DataFrame,
                                cfg: dict, ref_origin):
    """Deterministic georeference: start point from ground truth, heading from
    the measured PX4 attitude -- no least-squares fit to the GT track.

    Where `umeyama_alignment` finds both R and t by best-fitting the whole matched
    trajectory (so the GT shapes the result everywhere), this uses only two
    anchors:

      * translation -- the trajectory start is pinned to the GNSS ENU position at
        the first pose time (the *starting point* comes from ground truth);
      * rotation    -- body(FRD)->NED from `/fmu/out/vehicle_attitude` at the first
        scan, composed with NED->ENU, sets true north/east/south/west. With
        `lidar.body_frame` the odometry world frame *is* the Pixhawk FRD frame at
        t0 (poses seeded at identity), so the measured attitude rotates the whole
        local track straight onto the compass; without it, the LiDAR->body
        extrinsic is folded in first.

    Returns the same tuple as `align_and_georeference`. The reported RMSE is the
    same eph-weighted residual the umeyama path reports, so the two are directly
    comparable -- but note this fit does *not* minimise it.
    """
    from scipy.spatial.transform import Slerp
    from .attitude import _read_attitude, PX4_ATTITUDE_TOPIC

    lat0, lon0, alt0 = ref_origin
    gt_enu = geodetic_to_enu(gt_df["lat"].values, gt_df["lon"].values,
                             gt_df["alt"].values, lat0, lon0, alt0)

    # Rotation: measured attitude at the first scan (body FRD -> NED).
    topic = cfg.get("lidar", {}).get("attitude_topic", PX4_ATTITUDE_TOPIC)
    att_t, att_R = _read_attitude(cfg["paths"]["bag_dir"], topic)
    t0 = float(poses_df["timestamp"].iloc[0])
    t0c = float(np.clip(t0, att_t[0], att_t[-1]))
    R_body_to_ned = Slerp(att_t, att_R)([t0c])[0].as_matrix()

    # Compose measured attitude onto ENU: R = frame @ (body->NED). The odometry
    # world frame is NOT plain body-FRD -- with lidar.body_frame the scans go
    # through the LiDAR->body extrinsic before deskew, and empirically the world
    # frame that KISS-ICP tracks relates to NED->ENU by that same extrinsic on the
    # ENU side. Including it reproduces the GNSS-fit heading to ~1 deg on Test1
    # (where the attitude is known-good); omitting it leaves the heading ~150 deg
    # off with a flipped vertical. Without body_frame, fall back to plain NED->ENU.
    from .frames import build_lidar_to_body
    ext = build_lidar_to_body(cfg)
    R_frame = _ned_to_enu() if ext is None else np.asarray(ext)[:3, :3] @ _ned_to_enu()
    R = R_frame @ R_body_to_ned

    # Translation: pin the LiDAR start to the FIRST ROW of the ground truth (the
    # same point the odometry ENU frame is seeded at), not the time-matched GT
    # sample. gt_enu is relative to ref_origin, so this is ~(0,0,0) when the origin
    # is that first row.
    gt_start = gt_enu[0]
    odom_start = poses_df[["x", "y", "z"]].values[0]
    t = gt_start - R @ odom_start

    all_xyz = poses_df[["x", "y", "z"]].values
    aligned = (R @ all_xyz.T).T + t
    lat, lon, alt = enu_to_geodetic(aligned, lat0, lon0, alt0)

    # RMSE over the same matched, eph-weighted pairs as the umeyama path (report
    # only -- this georeference is not fit to them).
    q_idx, r_idx = match_poses_to_gt(poses_df, gt_df, cfg)
    w = match_weights(gt_df, r_idx, cfg)
    sq = np.sum((aligned[q_idx] - gt_enu[r_idx]) ** 2, axis=1)
    fit_rmse_m = float(np.sqrt(np.average(sq, weights=w) if w is not None
                               else np.mean(sq)))

    traj = pd.DataFrame({
        "timestamp": poses_df["timestamp"].values,
        "lat": lat, "lon": lon, "alt": alt,
        "x_enu": aligned[:, 0], "y_enu": aligned[:, 1], "z_enu": aligned[:, 2],
    })
    return traj, (lat0, lon0, alt0), fit_rmse_m, (R, t)


def anchored_alignment(poses_df: pd.DataFrame, gt_df: pd.DataFrame, cfg: dict,
                       ref_origin):
    """Pin the start points together and fit ONLY the rotation about that anchor.

    A middle ground between `umeyama_alignment` (fits R *and* t, so the start
    floats) and `attitude_anchored_alignment` (start pinned, rotation from the
    IMU): here the start is pinned to the first GT row (== the seeded ENU origin
    and the first odometry pose), and the rotation about that fixed start is the
    least-squares best fit to the rest of the matched GT track. There is no free
    translation, so the two trajectories always begin at exactly the same point.

    Returns the same tuple as `align_and_georeference`.
    """
    lat0, lon0, alt0 = ref_origin
    gt_enu = geodetic_to_enu(gt_df["lat"].values, gt_df["lon"].values,
                             gt_df["alt"].values, lat0, lon0, alt0)

    gt_start = gt_enu[0]                                   # first GT row
    odom_start = poses_df[["x", "y", "z"]].values[0]       # first odometry pose

    q_idx, r_idx = match_poses_to_gt(poses_df, gt_df, cfg)
    if len(q_idx) < 3:
        raise RuntimeError(
            f"Only {len(q_idx)} timestamp matches between SLAM poses and ground "
            f"truth. Check time windows / shared epoch / alignment.time_match.")

    # Fit the rotation about the shared start (subtract each side's own start).
    src = poses_df[["x", "y", "z"]].values[q_idx] - odom_start
    dst = gt_enu[r_idx] - gt_start
    w = match_weights(gt_df, r_idx, cfg)
    R = rotation_about_anchor(src, dst, w)
    t = gt_start - R @ odom_start                          # starts coincide exactly

    all_xyz = poses_df[["x", "y", "z"]].values
    aligned = (R @ all_xyz.T).T + t
    lat, lon, alt = enu_to_geodetic(aligned, lat0, lon0, alt0)

    sq = np.sum((aligned[q_idx] - gt_enu[r_idx]) ** 2, axis=1)
    fit_rmse_m = float(np.sqrt(np.average(sq, weights=w) if w is not None
                               else np.mean(sq)))

    traj = pd.DataFrame({
        "timestamp": poses_df["timestamp"].values,
        "lat": lat, "lon": lon, "alt": alt,
        "x_enu": aligned[:, 0], "y_enu": aligned[:, 1], "z_enu": aligned[:, 2],
    })
    return traj, (lat0, lon0, alt0), fit_rmse_m, (R, t)


def align_and_georeference(poses_df: pd.DataFrame, gt_df: pd.DataFrame, cfg: dict,
                           ref_origin=None):
    """Returns (traj_latlon_df, ref_origin, fit_rmse_m, (R, t)).

    Georeferences the odometry onto the GNSS ENU frame. The method is chosen by
    `alignment.method`:
      * "umeyama" (default) -- global least-squares rigid fit, optionally
        eph-weighted (both R and t from best-fitting the whole matched track);
      * "anchored" -- pin the start to the first GT row and fit only the rotation
        about it (see `anchored_alignment`);
      * "attitude_anchored" -- start point from ground truth, heading from the
        measured PX4 attitude (see `attitude_anchored_alignment`).

    If `ref_origin` (lat0, lon0, alt0) is given it is used as the ENU tangent
    point (so it matches the origin the odometry was seeded with); otherwise
    the ground truth's first sample is used.
    """
    if ref_origin is None:
        ref_origin = (float(gt_df["lat"].iloc[0]), float(gt_df["lon"].iloc[0]),
                      float(gt_df["alt"].iloc[0]))

    method = cfg.get("alignment", {}).get("method", "umeyama")
    if method == "attitude_anchored":
        return attitude_anchored_alignment(poses_df, gt_df, cfg, ref_origin)
    if method == "anchored":
        return anchored_alignment(poses_df, gt_df, cfg, ref_origin)
    if method != "umeyama":
        raise ValueError(
            f"Unknown alignment.method '{method}' (expected 'umeyama', "
            f"'anchored', or 'attitude_anchored').")

    lat0, lon0, alt0 = ref_origin

    gt_enu = geodetic_to_enu(gt_df["lat"].values, gt_df["lon"].values,
                             gt_df["alt"].values, lat0, lon0, alt0)

    q_idx, r_idx = match_poses_to_gt(poses_df, gt_df, cfg)
    if len(q_idx) < 10:
        raise RuntimeError(
            f"Only {len(q_idx)} timestamp matches between SLAM poses and ground "
            f"truth. Check time windows / shared epoch / alignment.time_match.")

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
