"""Geodetic (WGS84 lat/lon/alt) <-> local ENU (East-North-Up, meters)
conversions, used to bring the SLAM trajectory and the GNSS ground truth
into a common metric frame for alignment, and to project the aligned SLAM
trajectory back to lat/lon for the final deliverable.
"""
import numpy as np
import pymap3d as pm


def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0) -> np.ndarray:
    e, n, u = pm.geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
    return np.column_stack([e, n, u])


def enu_to_geodetic(enu: np.ndarray, lat0, lon0, alt0):
    lat, lon, alt = pm.enu2geodetic(enu[:, 0], enu[:, 1], enu[:, 2], lat0, lon0, alt0)
    return np.asarray(lat), np.asarray(lon), np.asarray(alt)
