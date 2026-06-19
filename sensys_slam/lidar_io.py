"""Load LiDAR scans for the KISS-ICP `register_frame` loop, either from the
per-frame `.laz` exports or by streaming the rosbag's `/ouster/points` topic
directly.

The bag-backed reader (`BagScanDataset`) exists because the `.laz` exports
lost the per-point sweep timing -- their `gps_time` field is all zeros -- so
deskewing is impossible from `.laz`. The raw PointCloud2 messages still carry
per-point time in a non-standard `timeoffset` field (milliseconds across the
~100 ms sweep), which the reader normalizes to [0, 1] so KISS-ICP can deskew.

Both datasets expose the same minimal interface used by `run_odometry`:
`len(dataset)` and `iter_scans()` yielding `(timestamp_s, points, point_times)`.
"""
from pathlib import Path

import numpy as np
import laspy

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from kiss_icp.tools.point_cloud2 import read_points

# Per-point sweep-time field names we know how to use, in priority order.
# Ouster's ROS2 PointCloud2 names it `timeoffset`; the others cover the
# conventional names so this also works on more standard clouds.
_POINT_TIME_FIELDS = ("timeoffset", "t", "time", "timestamp")


def _normalize_point_times(t: np.ndarray) -> np.ndarray:
    """Normalize per-point timestamps to [0, 1] across the scan, which is the
    convention KISS-ICP's deskewer expects (see kiss_icp.datasets.mulran).
    Returns all-zeros if there is no usable spread."""
    t = np.asarray(t, dtype=np.float64)
    if t.size == 0:
        return np.array([], dtype=np.float64)
    spread = t.max() - t.min()
    if spread <= 0:
        return np.zeros_like(t)
    return (t - t.min()) / spread


def read_laz_scan(filepath: str):
    """Read one `.laz` scan.

    Returns:
        points: (N, 3) float64 array of x, y, z in the Ouster sensor frame.
        point_times: (N,) float64 array of per-point relative timestamps in
            [0, 1] for deskewing, or an empty array if the LAS has no usable
            per-point time. The per-frame `.laz` exports in this dataset
            carry an all-zero `gps_time`, so this returns an empty array and
            deskewing is not possible from `.laz` -- use `BagScanDataset` for
            that. Set `kiss_icp.deskew: true` only with a source that
            provides real per-point time.
    """
    las = laspy.read(filepath)
    points = np.column_stack((las.x, las.y, las.z)).astype(np.float64)

    if "gps_time" in set(las.point_format.dimension_names):
        point_times = _normalize_point_times(np.asarray(las.gps_time, dtype=np.float64))
        # An all-zero / constant gps_time (this dataset's case) normalizes to
        # zeros, which carries no deskew information; signal "no time" instead.
        if np.any(point_times > 0):
            return points, point_times

    return points, np.array([], dtype=np.float64)


def read_pointcloud2_scan(msg, normalize: bool = True):
    """Read one PointCloud2 message into `(points, point_times)`.

    points: (N, 3) float64 in the sensor frame, NaN rows dropped.
    point_times: (N,) float64 from a `timeoffset` field (or the conventional
        t/time/timestamp names) when present, else an empty array. With
        `normalize=True` these are scaled to [0, 1] for KISS-ICP's deskewer;
        with `normalize=False` the raw field values are returned (e.g. Ouster
        `timeoffset` in milliseconds), as needed for external IMU deskew.
        NaN-xyz rows are dropped from both arrays together so points and times
        stay index-aligned.
    """
    field_names = {f.name for f in msg.fields}
    t_field = next((n for n in _POINT_TIME_FIELDS if n in field_names), None)
    wanted = ["x", "y", "z"] + ([t_field] if t_field else [])

    structured = read_points(msg, field_names=wanted)
    points = np.column_stack(
        [structured["x"], structured["y"], structured["z"]]
    ).astype(np.float64)
    keep = ~np.any(np.isnan(points), axis=1)
    points = points[keep]

    if t_field:
        raw = np.asarray(structured[t_field], dtype=np.float64)[keep]
        point_times = _normalize_point_times(raw) if normalize else raw
    else:
        point_times = np.array([], dtype=np.float64)
    return points, point_times


class LazScanDataset:
    """Sequence of `.laz` scans, indexed in manifest order.

    `manifest_df` must have columns: filepath, timestamp (see
    sensys_slam.timestamps.build_scan_manifest).
    """

    def __init__(self, manifest_df):
        self.manifest = manifest_df.reset_index(drop=True)

    def __len__(self):
        return len(self.manifest)

    def __getitem__(self, idx):
        row = self.manifest.iloc[idx]
        return read_laz_scan(row["filepath"])

    def scan_timestamp(self, idx) -> float:
        return float(self.manifest.iloc[idx]["timestamp"])

    def iter_scans(self):
        """Yield `(timestamp_s, points, point_times)` in manifest order."""
        for idx in range(len(self)):
            points, point_times = self[idx]
            yield self.scan_timestamp(idx), points, point_times


class BagScanDataset:
    """Stream LiDAR scans directly from the rosbag's `/ouster/points` topic.

    Unlike the `.laz` exports, the raw PointCloud2 messages still carry the
    per-point sweep timing (`timeoffset`), so this is the source to use when
    `kiss_icp.deskew: true`. Scan timestamps come from the bag-recorded
    message time (Unix seconds), identical to the manifest built in
    sensys_slam.timestamps.

    Scans are streamed sequentially (the odometry loop consumes them in
    order); there is no random access by index.

    If `deskewer` is given (an attitude.AttitudeDeskewer), each sweep is
    motion-compensated with measured attitude here and yielded with empty
    per-point times, so KISS-ICP's own (constant-velocity) deskew should be
    left off. The raw `timeoffset` field is assumed to be in milliseconds.
    """

    def __init__(self, bag_dir: str, topic: str, deskewer=None):
        self.bag_dir = bag_dir
        self.topic = topic
        self.deskewer = deskewer
        self._typestore = get_typestore(Stores.LATEST)
        with AnyReader([Path(bag_dir)], default_typestore=self._typestore) as reader:
            conns = [c for c in reader.connections if c.topic == topic]
            if not conns:
                available = sorted({c.topic for c in reader.connections})
                raise ValueError(
                    f"Topic '{topic}' not found in bag at {bag_dir}.\n"
                    f"Available topics:\n  " + "\n  ".join(available)
                )
            self._n = sum(c.msgcount for c in conns)

    def __len__(self):
        return self._n

    def iter_scans(self):
        """Yield `(timestamp_s, points, point_times)` in bag (capture) order."""
        with AnyReader([Path(self.bag_dir)], default_typestore=self._typestore) as reader:
            conns = [c for c in reader.connections if c.topic == self.topic]
            for connection, timestamp_ns, rawdata in reader.messages(connections=conns):
                msg = reader.deserialize(rawdata, connection.msgtype)
                t_s = timestamp_ns * 1e-9
                if self.deskewer is not None:
                    # External IMU-attitude deskew: raw timeoffset is in ms.
                    points, raw_t = read_pointcloud2_scan(msg, normalize=False)
                    points = self.deskewer.deskew(points, raw_t / 1000.0, t_s)
                    yield t_s, points, np.array([], dtype=np.float64)
                else:
                    points, point_times = read_pointcloud2_scan(msg)
                    yield t_s, points, point_times
