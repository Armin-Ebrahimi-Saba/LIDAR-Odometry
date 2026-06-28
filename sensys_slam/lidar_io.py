"""Load LiDAR scans for the odometry loop, either from the per-frame `.laz`
exports or by streaming the rosbag's `/ouster/points` topic directly.

The bag-backed reader (`BagScanDataset`) exists because the `.laz` exports lost
the per-point sweep timing (their `gps_time` is all zeros), so deskewing is
impossible from `.laz`. The raw PointCloud2 messages still carry per-point time
in a non-standard `timeoffset` field (milliseconds across the ~100 ms sweep),
which the reader normalizes to [0, 1] for the constant-velocity deskewer.

Both datasets expose the same minimal interface used by `run_odometry`:
`len(dataset)` and `iter_scans()` yielding `(timestamp_s, points, point_times)`.

PointCloud2 parsing is done here with numpy (no external dependency): a
structured dtype is built from the message's `fields` and `point_step`.
"""
from pathlib import Path

import numpy as np
import laspy
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

# sensor_msgs/PointField datatype code -> numpy dtype
_PF_TO_NP = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}
# Per-point sweep-time field names we know how to use, in priority order.
_POINT_TIME_FIELDS = ("timeoffset", "t", "time", "timestamp")


def read_points(msg, field_names):
    """Parse a sensor_msgs/PointCloud2 into a numpy structured array containing
    the requested `field_names`. Honors per-field offsets and the point stride."""
    by_name = {f.name: f for f in msg.fields}
    names, formats, offsets = [], [], []
    for name in field_names:
        if name not in by_name:
            continue
        f = by_name[name]
        np_t = np.dtype(_PF_TO_NP[f.datatype])
        if msg.is_bigendian:
            np_t = np_t.newbyteorder(">")
        names.append(name)
        formats.append(np_t)
        offsets.append(f.offset)

    dtype = np.dtype({"names": names, "formats": formats,
                      "offsets": offsets, "itemsize": msg.point_step})
    n = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=n)
    return raw


def _normalize_point_times(t: np.ndarray) -> np.ndarray:
    """Normalize per-point timestamps to [0, 1] across the scan. Returns
    all-zeros if there is no usable spread."""
    t = np.asarray(t, dtype=np.float64)
    if t.size == 0:
        return np.array([], dtype=np.float64)
    spread = t.max() - t.min()
    if spread <= 0:
        return np.zeros_like(t)
    return (t - t.min()) / spread


def read_pointcloud2_scan(msg, normalize: bool = True):
    """Read one PointCloud2 into `(points, point_times)`.

    points: (N, 3) float64 in the sensor frame, NaN rows dropped.
    point_times: (N,) float64 from a `timeoffset`/t/time/timestamp field if
        present, else empty. With `normalize=True` scaled to [0, 1]; with
        `normalize=False` the raw values (e.g. Ouster `timeoffset` in ms).
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


def read_laz_scan(filepath: str):
    """Read one `.laz` scan -> (points (N,3) float64, point_times).

    The per-frame `.laz` exports carry an all-zero `gps_time`, so per-point
    times normalize to zeros (no deskew info) and an empty array is returned;
    use `BagScanDataset` for deskewing.
    """
    las = laspy.read(filepath)
    points = np.column_stack((las.x, las.y, las.z)).astype(np.float64)
    if "gps_time" in set(las.point_format.dimension_names):
        point_times = _normalize_point_times(np.asarray(las.gps_time, dtype=np.float64))
        if np.any(point_times > 0):
            return points, point_times
    return points, np.array([], dtype=np.float64)


class LazScanDataset:
    """Sequence of `.laz` scans, indexed in manifest order.

    `manifest_df` must have columns: filepath, timestamp (see
    sensys_slam.timestamps.build_scan_manifest).
    """

    def __init__(self, manifest_df, frame_start=0, frame_end=None):
        m = manifest_df.reset_index(drop=True)
        end = len(m) - 1 if frame_end is None else min(int(frame_end), len(m) - 1)
        self.manifest = m.iloc[int(frame_start or 0):end + 1].reset_index(drop=True)

    def __len__(self):
        return len(self.manifest)

    def scan_timestamp(self, idx) -> float:
        return float(self.manifest.iloc[idx]["timestamp"])

    def iter_scans(self):
        for idx in range(len(self)):
            points, point_times = read_laz_scan(self.manifest.iloc[idx]["filepath"])
            yield self.scan_timestamp(idx), points, point_times


class BagScanDataset:
    """Stream LiDAR scans directly from the rosbag's `/ouster/points` topic.

    The raw PointCloud2 messages carry per-point sweep timing (`timeoffset`),
    normalized to [0, 1] and yielded so KISS-ICP's constant-velocity deskew can
    use it. Scan timestamps are the bag-recorded message times (Unix seconds).

    If `deskewer` is given (an attitude.AttitudeDeskewer), each sweep is instead
    rotation-deskewed here with measured PX4 attitude and yielded with empty
    per-point times (KISS-ICP's own deskew should then be left off). The raw
    `timeoffset` field is assumed to be in milliseconds.

    `frame_start`/`frame_end` select a closed range of scan indices to process
    (`frame_end` inclusive; None = until the last scan).
    """

    def __init__(self, bag_dir: str, topic: str, deskewer=None, frame_start=0, frame_end=None):
        self.bag_dir = bag_dir
        self.topic = topic
        self.deskewer = deskewer
        self.frame_start = int(frame_start or 0)
        self.frame_end = None if frame_end is None else int(frame_end)
        self._typestore = get_typestore(Stores.LATEST)
        with AnyReader([Path(bag_dir)], default_typestore=self._typestore) as reader:
            conns = [c for c in reader.connections if c.topic == topic]
            if not conns:
                available = sorted({c.topic for c in reader.connections})
                raise ValueError(
                    f"Topic '{topic}' not found in bag at {bag_dir}.\n"
                    f"Available topics:\n  " + "\n  ".join(available)
                )
            total = sum(c.msgcount for c in conns)
        last = total - 1 if self.frame_end is None else min(self.frame_end, total - 1)
        self._n = max(0, last - self.frame_start + 1)

    def __len__(self):
        return self._n

    def iter_scans(self):
        with AnyReader([Path(self.bag_dir)], default_typestore=self._typestore) as reader:
            conns = [c for c in reader.connections if c.topic == self.topic]
            for i, (connection, t_ns, rawdata) in enumerate(reader.messages(connections=conns)):
                if i < self.frame_start:
                    continue
                if self.frame_end is not None and i > self.frame_end:
                    break
                msg = reader.deserialize(rawdata, connection.msgtype)
                t_s = t_ns * 1e-9
                if self.deskewer is not None:
                    points, raw_t = read_pointcloud2_scan(msg, normalize=False)
                    points = self.deskewer.deskew(points, raw_t / 1000.0, t_s)
                    yield t_s, points, np.array([], dtype=np.float64)
                else:
                    points, point_times = read_pointcloud2_scan(msg)
                    yield t_s, points, point_times
