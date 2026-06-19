"""Pair sorted .laz scan files with bag-recorded timestamps.

The rosbag's .db3 stores, for every message, both the serialized payload and
the time at which it was recorded by the logger (nanoseconds since Unix
epoch). We only need the latter to time-tag each .laz scan, so we iterate
the bag's `/ouster/points` connection and read the recorded timestamp for
each message *without* deserializing the (large) PointCloud2 payload. This
keeps the step fast and memory-light even though the bag itself is tens of
GB.

Assumption: the Nth .laz file (sorted by filename) corresponds to the Nth
`/ouster/points` message in the bag, in chronological order. This holds as
long as the .laz files were exported in capture order, which is the normal
case for sequential per-frame dumps. The manifest builder below raises an
error rather than guessing silently if the message count does not match the
number of .laz files, so a mismatch will surface immediately instead of
silently mis-pairing data.
"""
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def extract_topic_timestamps(bag_dir: str, topic: str) -> np.ndarray:
    """Return bag-recorded timestamps (Unix seconds, float64) for every
    message on `topic`, in the order they appear in the bag."""
    bag_path = Path(bag_dir)
    timestamps_ns = []
    typestore = get_typestore(Stores.LATEST)
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            available = sorted({c.topic for c in reader.connections})
            raise ValueError(
                f"Topic '{topic}' not found in bag at {bag_path}.\n"
                f"Available topics:\n  " + "\n  ".join(available)
            )
        for _connection, timestamp_ns, _rawdata in reader.messages(connections=connections):
            timestamps_ns.append(timestamp_ns)

    if not timestamps_ns:
        raise RuntimeError(f"No messages found for topic '{topic}' in {bag_path}")

    return np.asarray(timestamps_ns, dtype=np.int64) * 1e-9  # ns -> unix seconds


def build_scan_manifest(bag_dir: str, laz_dir: str, topic: str, out_csv: str) -> pd.DataFrame:
    """Build and save a manifest CSV with columns: filename, filepath, timestamp.

    Raises a RuntimeError (rather than guessing) if the number of .laz files
    does not match the number of messages on `topic` in the bag.
    """
    laz_files = sorted(Path(laz_dir).glob("*.laz"))
    if not laz_files:
        raise FileNotFoundError(f"No .laz files found in {laz_dir}")

    timestamps = extract_topic_timestamps(bag_dir, topic)

    if len(timestamps) != len(laz_files):
        raise RuntimeError(
            f"Mismatch building scan manifest: {len(laz_files)} .laz files in "
            f"{laz_dir} vs {len(timestamps)} '{topic}' messages in {bag_dir}.\n"
            f"Double-check that bag_dir, laz_dir and topic in config.yaml all "
            f"refer to the same run before proceeding -- pairing mismatched "
            f"counts would silently corrupt every downstream timestamp."
        )

    df = pd.DataFrame(
        {
            "filename": [f.name for f in laz_files],
            "filepath": [str(f) for f in laz_files],
            "timestamp": timestamps,
        }
    )
    Path(out_csv).parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out_csv, index=False)
    print(f"[timestamps] wrote {len(df)} scan timestamps -> {out_csv}")
    print(f"[timestamps] window: {df['timestamp'].min():.3f} -> {df['timestamp'].max():.3f} "
          f"({df['timestamp'].max() - df['timestamp'].min():.1f} s)")
    return df
