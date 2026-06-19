"""Compute NED-frame velocity from the aligned ENU trajectory via finite
differences.

Raw frame-to-frame SLAM jitter (typically a few cm) gets amplified by naive
differentiation at ~10 Hz into tens of cm/s of velocity noise, even though
the *positions* themselves are accurate -- this is expected, not a bug, and
gets worse the noisier the input positions are. A light Savitzky-Golay
smoothing pass (applied to position before differentiating) tames this
without needing a real filter/EKF; disable it (smooth_window=0) if you'd
rather difference the raw aligned positions directly.
"""
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter


def compute_ned_velocity(
    traj_latlon_df: pd.DataFrame, smooth_window: int = 11, smooth_polyorder: int = 2
) -> pd.DataFrame:
    """Add vel_n_m_s, vel_e_m_s, vel_d_m_s columns to a copy of the input
    trajectory dataframe (which must have timestamp, x_enu, y_enu, z_enu).

    smooth_window: odd window length (in samples) for Savitzky-Golay
        position smoothing before differentiating. Set to 0 to disable and
        differentiate the raw positions directly.
    """
    t = traj_latlon_df["timestamp"].values
    e = traj_latlon_df["x_enu"].values
    n = traj_latlon_df["y_enu"].values
    u = traj_latlon_df["z_enu"].values

    if len(t) < 2:
        raise RuntimeError("Need at least 2 trajectory points to compute velocity.")

    if smooth_window and smooth_window > 2 and len(t) > smooth_window:
        window = smooth_window if smooth_window % 2 == 1 else smooth_window + 1
        e = savgol_filter(e, window, smooth_polyorder)
        n = savgol_filter(n, window, smooth_polyorder)
        u = savgol_filter(u, window, smooth_polyorder)

    vel_e = np.gradient(e, t)
    vel_n = np.gradient(n, t)
    vel_u = np.gradient(u, t)

    out = traj_latlon_df.copy()
    out["vel_n_m_s"] = vel_n
    out["vel_e_m_s"] = vel_e
    out["vel_d_m_s"] = -vel_u  # NED down = -ENU up
    return out
