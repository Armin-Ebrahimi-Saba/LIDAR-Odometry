#!/usr/bin/env python3
"""Convert a recorded LIO-SAM odometry bag into this pipeline's poses_local.csv.

LIO_SAM_6AXIS publishes its optimized pose as ``nav_msgs/Odometry`` (per scan).
We record that to a small ROS1 bag during the run; this script reads it and
writes ``poses_local.csv`` (timestamp,x,y,z,qx,qy,qz,qw) -- the exact schema the
existing align/velocity/evaluate stages consume, so they run unchanged
afterwards (see README.md and run_pipeline.py).

Usage:
    python scripts/odom_to_poses.py outputs/test1_liosam/lio_odom.bag \
        --out outputs/test1_liosam/poses_local.csv
"""
import argparse
from pathlib import Path

import pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

ODOM_TYPE = "nav_msgs/msg/Odometry"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="ROS1 bag containing the LIO-SAM odometry topic")
    ap.add_argument("--topic", default=None,
                    help="Odometry topic (default: auto-detect, preferring mapping/odometry)")
    ap.add_argument("--out", default="outputs/test1_liosam/poses_local.csv")
    args = ap.parse_args()

    ts = get_typestore(Stores.ROS1_NOETIC)
    with AnyReader([Path(args.bag)], default_typestore=ts) as reader:
        odom_conns = [c for c in reader.connections if c.msgtype == ODOM_TYPE]
        if not odom_conns:
            raise SystemExit(f"No {ODOM_TYPE} topics in {args.bag}. "
                             f"Topics present: {[c.topic for c in reader.connections]}")
        if args.topic:
            conns = [c for c in odom_conns if c.topic == args.topic]
            if not conns:
                raise SystemExit(f"Topic {args.topic} not found. "
                                 f"Odometry topics: {[c.topic for c in odom_conns]}")
        else:
            # Prefer the final optimized topic over the incremental one.
            ranked = sorted(odom_conns, key=lambda c: (
                "incremental" in c.topic, "mapping" not in c.topic, c.topic))
            conns = [ranked[0]]
            print(f"[odom] auto-selected topic: {conns[0].topic} "
                  f"(candidates: {[c.topic for c in odom_conns]})")

        rows = []
        for conn, t, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            st = m.header.stamp
            stamp = st.sec + st.nanosec * 1e-9
            if stamp <= 0:
                stamp = t * 1e-9
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            rows.append((stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    df = pd.DataFrame(rows, columns=["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
    df = df.drop_duplicates(subset="timestamp").sort_values("timestamp").reset_index(drop=True)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out, index=False)
    span = df["timestamp"].iloc[-1] - df["timestamp"].iloc[0] if len(df) else 0
    print(f"[odom] wrote {len(df)} poses spanning {span:.1f} s -> {out}")


if __name__ == "__main__":
    main()
