#!/usr/bin/env python3
"""Extract PX4 attitude quaternions (for scan deskewing) from the bag to CSV.

KISS-ICP deskews each sweep assuming constant angular velocity; this project
can instead use the platform's measured attitude (see sensys_slam/attitude.py).
The source is `/fmu/out/vehicle_attitude` (PX4 EKF, ~100 Hz) -- NOT
`/ouster/imu_att`, which is all-identity in this bag and unusable.

The bag does not embed the message definition, so px4_msgs/msg/VehicleAttitude
is registered here from its field layout. Quaternions are stored PX4-Hamilton
order q(w, x, y, z); this writes both that and the scipy (x, y, z, w) order.
Output is cropped to the run window (+/- margin so the edge sweeps still have
attitude on both sides to interpolate from).

Usage:
    python scripts/extract_attitude.py [--config config.yaml] [--margin 1.0] [--output PATH]
"""
import argparse
from pathlib import Path

import pandas as pd
import yaml
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

DEFAULT_ATTITUDE_TOPIC = "/fmu/out/vehicle_attitude"

# px4_msgs/msg/VehicleAttitude -- field order/types must match the recorded
# message for CDR decoding. q is Hamilton order [w, x, y, z].
_VEHICLE_ATTITUDE_MSG = """
uint64 timestamp
uint64 timestamp_sample
float32[4] q
float32[4] delta_q_reset
uint8 quat_reset_counter
"""


def _typestore_with_attitude():
    ts = get_typestore(Stores.LATEST)
    ts.register(get_types_from_msg(_VEHICLE_ATTITUDE_MSG, "px4_msgs/msg/VehicleAttitude"))
    return ts


def extract_attitude(config_path: str, margin_s: float = 1.0, output_path: str | None = None) -> Path:
    cfg = yaml.safe_load(Path(config_path).read_text())
    bag_dir = Path(cfg["paths"]["bag_dir"])
    t0 = float(cfg["run"]["start_time"]) - margin_s
    t1 = float(cfg["run"]["end_time"]) + margin_s
    run_name = str(cfg["run"].get("name", "run")).lower()
    topic = cfg.get("lidar", {}).get("attitude_topic", DEFAULT_ATTITUDE_TOPIC)

    rows = []
    ts = _typestore_with_attitude()
    with AnyReader([bag_dir], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            available = sorted({c.topic for c in reader.connections})
            raise SystemExit(
                f"Attitude topic '{topic}' not found in {bag_dir}.\n"
                "Available topics:\n  " + "\n  ".join(available)
            )
        for conn, t_ns, raw in reader.messages(connections=conns):
            t = t_ns * 1e-9
            if t < t0 or t > t1:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            qw, qx, qy, qz = (float(v) for v in msg.q)
            # Skip uninitialized/invalid samples (non-unit quaternions).
            if not 0.98 < (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5 < 1.02:
                continue
            rows.append((t, msg.timestamp * 1e-6, msg.timestamp_sample * 1e-6,
                         qw, qx, qy, qz, int(msg.quat_reset_counter)))

    if not rows:
        raise SystemExit(
            f"No valid '{topic}' samples in [{t0}, {t1}]. Check the run window "
            f"and that the topic carries real (non-identity) attitude."
        )

    df = pd.DataFrame(rows, columns=[
        "bag_time", "timestamp", "timestamp_sample",
        "qw", "qx", "qy", "qz", "quat_reset_counter",
    ]).sort_values("bag_time").reset_index(drop=True)
    # scipy/ROS (x, y, z, w) order alongside the PX4 (w, x, y, z) columns.
    df["qx_s"], df["qy_s"], df["qz_s"], df["qw_s"] = df.qx, df.qy, df.qz, df.qw

    out = Path(output_path) if output_path else bag_dir.parent / f"imu_attitude_{run_name}.csv"
    df.to_csv(out, index=False)

    span = df.bag_time.max() - df.bag_time.min()
    print(f"[extract_attitude] topic: {topic}  bag: {bag_dir}")
    print(f"[extract_attitude] {run_name}: {len(df)} samples over {span:.1f}s "
          f"(~{len(df)/span:.0f} Hz), window [{t0:.3f}, {t1:.3f}] (margin {margin_s}s)")
    print(f"[extract_attitude] reset counters: {sorted(df.quat_reset_counter.unique())}")
    print(f"[extract_attitude] wrote {out}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="config.yaml", help="path to config.yaml")
    ap.add_argument("--margin", type=float, default=1.0,
                    help="seconds added before/after the run window (default 1.0)")
    ap.add_argument("--output", default=None, help="output CSV path (default: data/imu_attitude_<run>.csv)")
    args = ap.parse_args()
    extract_attitude(args.config, args.margin, args.output)


if __name__ == "__main__":
    main()
