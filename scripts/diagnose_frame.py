#!/usr/bin/env python3
"""Diagnose whether /ouster/points in the bag is sensor-frame (moves with
the vehicle) or already a fixed/world frame (frozen scene), independent of
whatever the .laz export did.

Usage:
    python diagnose_frame.py ./data/rosbag
"""
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from kiss_icp.tools.point_cloud2 import read_point_cloud

bag_dir = sys.argv[1]
topic = sys.argv[2] if len(sys.argv) > 2 else "/ouster/points"

typestore = get_typestore(Stores.LATEST)
with AnyReader([Path(bag_dir)], default_typestore=typestore) as reader:
    connections = [c for c in reader.connections if c.topic == topic]
    if not connections:
        print(f"Topic '{topic}' not found.")
        sys.exit(1)

    msgs = reader.messages(connections=connections)

    # --- check 1: frame_id ---
    connection, t0_ns, raw0 = next(msgs)
    msg0 = reader.deserialize(raw0, connection.msgtype)
    print(f"frame_id of first message: '{msg0.header.frame_id}'")
    print("  -> if this looks like 'map', 'odom', 'world', 'local_origin', etc: FIXED FRAME (bug confirmed upstream)")
    print("  -> if this looks like 'os_sensor', 'os_lidar', 'lidar_link', 'base_link': SENSOR FRAME (good)")

    # --- check 2: does the raw bag cloud actually shift between two scans, ~10s apart? ---
    pts0, _ = read_point_cloud(msg0)
    print(f"\nfirst scan: {len(pts0)} points, centroid {pts0.mean(axis=0).round(2)}")

    target_gap_ns = int(10e9)  # 10 seconds later
    msg1 = None
    for connection, t_ns, raw in msgs:
        if t_ns - t0_ns >= target_gap_ns:
            msg1 = reader.deserialize(raw, connection.msgtype)
            break

    if msg1 is None:
        print("Bag too short for a 10s-later comparison; reduce target_gap_ns.")
        sys.exit(0)

    pts1, _ = read_point_cloud(msg1)
    print(f"scan ~10s later: {len(pts1)} points, centroid {pts1.mean(axis=0).round(2)}")

    centroid_shift = np.linalg.norm(pts1.mean(axis=0) - pts0.mean(axis=0))
    print(f"\ncentroid shift over ~10s: {centroid_shift:.2f} m")
    print("  -> near 0: scene is frozen relative to the cloud's own frame (matches your finding)")
    print("  -> clearly nonzero and growing with the gap: scene genuinely shifts (sensor-frame data)")