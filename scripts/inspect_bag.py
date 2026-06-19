#!/usr/bin/env python3
"""Quick inspection utility: list topics, message types, and message counts
in a ROS2 bag, without needing ROS2 installed.

Run this FIRST, before anything else in the pipeline, to confirm that the
topic names and message types in config.yaml actually match your bag --
especially important for the optional IMU-assist module, whose message
field assumptions were inferred from the inventory report, not the data
itself.

Usage:
    python scripts/inspect_bag.py "./data/rosbag/<bag folder>/meta"
"""
import argparse
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", help="Path to the folder containing metadata.yaml + rosbag_0.db3")
    args = parser.parse_args()

    typestore = get_typestore(Stores.LATEST)
    with AnyReader([Path(args.bag_dir)], default_typestore=typestore) as reader:
        print(f"{'topic':40s} {'msgtype':45s} {'count':>8s}")
        print("-" * 95)
        for c in sorted(reader.connections, key=lambda c: c.topic):
            print(f"{c.topic:40s} {c.msgtype:45s} {c.msgcount:8d}")
        print("-" * 95)
        print(f"start: {reader.start_time * 1e-9:.3f}   end: {reader.end_time * 1e-9:.3f}   "
              f"duration: {(reader.end_time - reader.start_time) * 1e-9:.1f} s")


if __name__ == "__main__":
    main()
