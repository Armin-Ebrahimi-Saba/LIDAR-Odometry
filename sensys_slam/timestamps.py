"""Pair sorted .laz scan files with bag-recorded timestamps.

The rosbag's .db3 stores, for every message, the time at which it was recorded
(nanoseconds since Unix epoch). We iterate the `/ouster/points` connection and
read that recorded timestamp for each message *without* deserializing the
(large) PointCloud2 payload, keeping the step fast even on a tens-of-GB bag.

Assumption: the Nth .laz file (sorted by filename) corresponds to the Nth
`/ouster/points` message, in chronological order. The builder raises rather
than guessing if the counts do not match, so a mismatch surfaces immediately
instead of silently mis-pairing data.
"""
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def extract_topic_timestamps(bag_dir: str, topic: str) -> np.ndarray:
    """Return bag-recorded timestamps (Unix seconds) for every message on
    `topic`, in bag order."""
    timestamps_ns = []
    typestore = get_typestore(Stores.LATEST)
    with AnyReader([Path(bag_dir)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            available = sorted({c.topic for c in reader.connections})
            raise ValueError(
                f"Topic '{topic}' not found in bag at {bag_dir}.\n"
                f"Available topics:\n  " + "\n  ".join(available)
            )
        for _conn, t_ns, _raw in reader.messages(connections=connections):
            timestamps_ns.append(t_ns)

    if not timestamps_ns:
        raise RuntimeError(f"No messages found for topic '{topic}' in {bag_dir}")
    return np.asarray(timestamps_ns, dtype=np.int64) * 1e-9


def build_scan_manifest(bag_dir: str, laz_dir: str, topic: str, out_csv: str) -> pd.DataFrame:
    """Build/save a manifest CSV with columns: filename, filepath, timestamp."""
    laz_files = sorted(Path(laz_dir).glob("*.laz"))
    if not laz_files:
        raise FileNotFoundError(f"No .laz files found in {laz_dir}")

    timestamps = extract_topic_timestamps(bag_dir, topic)
    if len(timestamps) != len(laz_files):
        raise RuntimeError(
            f"Mismatch building scan manifest: {len(laz_files)} .laz files in "
            f"{laz_dir} vs {len(timestamps)} '{topic}' messages in {bag_dir}.\n"
            f"Check that bag_dir, laz_dir and topic all refer to the same run."
        )

    df = pd.DataFrame({
        "filename": [f.name for f in laz_files],
        "filepath": [str(f) for f in laz_files],
        "timestamp": timestamps,
    })
    Path(out_csv).parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out_csv, index=False)
    print(f"[timestamps] wrote {len(df)} scan timestamps -> {out_csv}")
    print(f"[timestamps] window: {df['timestamp'].min():.3f} -> {df['timestamp'].max():.3f} "
          f"({df['timestamp'].max() - df['timestamp'].min():.1f} s)")
    return df
