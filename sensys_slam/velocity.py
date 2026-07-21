"""NED-frame velocity from the georeferenced trajectory.

The trajectory carries local ENU coordinates (x_enu=East, y_enu=North,
z_enu=Up). Velocity is the time-derivative of position; positions are
optionally smoothed first (Savitzky-Golay) to suppress per-scan jitter before
differentiating. NED = (North, East, Down), so:
    vel_n = d(North)/dt,  vel_e = d(East)/dt,  vel_d = -d(Up)/dt
"""
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter


def _smooth(x: np.ndarray, window: int, polyorder: int) -> np.ndarray:
    if window and window > polyorder and window <= len(x):
        w = window if window % 2 == 1 else window + 1
        return savgol_filter(x, w, polyorder)
    return x


def compute_ned_velocity(traj_df: pd.DataFrame, smooth_window: int = 11,
                         smooth_polyorder: int = 2) -> pd.DataFrame:
    df = traj_df.sort_values("timestamp").reset_index(drop=True).copy()
    t = df["timestamp"].values.astype(np.float64)

    east = _smooth(df["x_enu"].values.astype(np.float64), smooth_window, smooth_polyorder)
    north = _smooth(df["y_enu"].values.astype(np.float64), smooth_window, smooth_polyorder)
    up = _smooth(df["z_enu"].values.astype(np.float64), smooth_window, smooth_polyorder)

    df["vel_n_m_s"] = np.gradient(north, t)
    df["vel_e_m_s"] = np.gradient(east, t)
    df["vel_d_m_s"] = -np.gradient(up, t)
    df["speed_m_s"] = np.sqrt(df.vel_n_m_s**2 + df.vel_e_m_s**2 + df.vel_d_m_s**2)
    return df
