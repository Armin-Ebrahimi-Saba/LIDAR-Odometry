"""Load individual .laz point cloud scans and expose them through a minimal
sequence interface compatible with the KISS-ICP `register_frame` loop.
"""
import numpy as np
import laspy


def read_laz_scan(filepath: str):
    """Read one .laz scan.

    Returns:
        points: (N, 3) float64 array of x, y, z in the Ouster sensor frame.
        point_times: (N,) float64 array of per-point relative timestamps in
            [0, 1] (for KISS-ICP motion deskewing), or an empty array if the
            LAS point format has no `gps_time` dimension. Most per-frame
            .laz exports do not carry per-point time, in which case KISS-ICP
            simply treats the scan as already deskewed -- set
            kiss_icp.deskew: true in config.yaml only if you have confirmed
            your .laz files do carry a usable gps_time field.
    """
    las = laspy.read(filepath)
    points = np.column_stack((las.x, las.y, las.z)).astype(np.float64)

    dim_names = set(las.point_format.dimension_names)
    if "gps_time" in dim_names:
        t = np.asarray(las.gps_time, dtype=np.float64)
        spread = t.max() - t.min()
        point_times = (t - t.min()) / spread if spread > 0 else np.zeros_like(t)
        return points, point_times

    return points, np.array([], dtype=np.float64)


class LazScanDataset:
    """Sequence of .laz scans, indexed in manifest order.

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
