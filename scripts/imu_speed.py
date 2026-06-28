#!/usr/bin/env python3
"""Report the IMU-based speed of the platform at a given LiDAR frame.

The raw IMU only measures acceleration / angular rate, so "speed from the IMU"
means the PX4 EKF velocity state, which is IMU-propagated and GPS-corrected.
That estimate is published on `/fmu/out/vehicle_local_position` (NED `vx,vy,vz`)
and mirrored on `/fmu/out/vehicle_odometry` (`velocity[3]`).

This script:
  1. finds the bag-record time of LiDAR frame N (the Nth `/ouster/points` msg),
  2. reads the nearest PX4 EKF velocity sample, and
  3. prints the velocity vector and the speed |v|.

The bag stores px4_msgs as raw CDR without embedded definitions, so the message
layout is registered from the px4_msgs clone (`--px4-msgs-dir`, default
./px4_msgs; see DESCRIPTION.md for cloning it).

Usage:
    python scripts/imu_speed.py 1                 # speed at LiDAR frame 1
    python scripts/imu_speed.py 1500 --source odometry
    python scripts/imu_speed.py 0 --config config.yaml --px4-msgs-dir ./px4_msgs
"""
import argparse
from pathlib import Path

import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# topic + velocity accessor for each --source
_SOURCES = {
    "local_position": (
        "/fmu/out/vehicle_local_position_v1",
        "px4_msgs/msg/VehicleLocalPosition",
        lambda m: (m.vx, m.vy, m.vz),
    ),
    "odometry": (
        "/fmu/out/vehicle_odometry",
        "px4_msgs/msg/VehicleOdometry",
        lambda m: tuple(m.velocity),
    ),
}


def _typestore(px4_msgs_dir: Path, type_name: str):
    """Register the requested px4_msgs type from the local clone."""
    name = type_name.rsplit("/", 1)[-1]            # e.g. VehicleLocalPosition
    msg_file = px4_msgs_dir / "msg" / f"{name}.msg"
    if not msg_file.exists():
        raise SystemExit(
            f"Cannot find {msg_file}. Clone px4_msgs (release/1.17) or pass "
            f"--px4-msgs-dir (see DESCRIPTION.md)."
        )
    ts = get_typestore(Stores.LATEST)
    ts.register(get_types_from_msg(msg_file.read_text(), type_name))
    return ts


def _lidar_frame_time(bag_dir: str, topic: str, frame: int, typestore) -> float:
    """Bag-record time (s) of the Nth message on `topic`."""
    with AnyReader([Path(bag_dir)], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            available = sorted({c.topic for c in reader.connections})
            raise SystemExit(
                f"Topic '{topic}' not found in {bag_dir}.\n"
                "Available topics:\n  " + "\n  ".join(available)
            )
        total = sum(c.msgcount for c in conns)
        if frame < 0 or frame >= total:
            raise SystemExit(f"Frame {frame} out of range (topic has {total} messages).")
        for i, (_c, t_ns, _raw) in enumerate(reader.messages(connections=conns)):
            if i == frame:
                return t_ns * 1e-9
    raise SystemExit(f"Could not reach frame {frame} on '{topic}'.")


def imu_speed(config_path: str, frame: int, source: str = "local_position",
              px4_msgs_dir: str = "./px4_msgs", lidar_topic: str | None = None):
    """Return (speed_m_s, velocity_xyz, sample_time_s, frame_time_s)."""
    cfg = yaml.safe_load(Path(config_path).read_text())
    bag_dir = cfg["paths"]["bag_dir"]
    topic = lidar_topic or cfg["run"].get("lidar_topic", "/ouster/points")

    vel_topic, type_name, getter = _SOURCES[source]
    ts = _typestore(Path(px4_msgs_dir), type_name)

    t_frame = _lidar_frame_time(bag_dir, topic, frame, ts)

    best_v, best_t = None, None
    with AnyReader([Path(bag_dir)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == vel_topic]
        if not conns:
            raise SystemExit(f"Velocity topic '{vel_topic}' not found in {bag_dir}.")
        for _c, t_ns, raw in reader.messages(connections=conns):
            tt = t_ns * 1e-9
            if best_t is None or abs(tt - t_frame) < abs(best_t - t_frame):
                best_v = getter(reader.deserialize(raw, type_name))
                best_t = tt
            elif tt > t_frame + 1.0:        # past the target; nearest already found
                break
    v = np.asarray(best_v, dtype=float)
    return float(np.linalg.norm(v)), v, best_t, t_frame


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frame", type=int, help="LiDAR frame index (Nth /ouster/points message)")
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--source", default="local_position", choices=list(_SOURCES),
                    help="PX4 EKF velocity source (default: local_position)")
    ap.add_argument("--px4-msgs-dir", default="./px4_msgs",
                    help="path to the px4_msgs clone (default: ./px4_msgs)")
    ap.add_argument("--topic", default=None, help="override LiDAR topic")
    args = ap.parse_args()

    speed, v, t_sample, t_frame = imu_speed(
        args.config, args.frame, args.source, args.px4_msgs_dir, args.topic)

    print(f"IMU/EKF speed at LiDAR frame {args.frame}  (source: {args.source})")
    print(f"  LiDAR frame time : {t_frame:.3f} s")
    print(f"  EKF sample time  : {t_sample:.3f} s  (Δ={t_sample - t_frame:+.3f}s)")
    print(f"  velocity (NED)   : [{v[0]:+.3f}, {v[1]:+.3f}, {v[2]:+.3f}] m/s")
    print(f"  horizontal speed : {np.hypot(v[0], v[1]):.3f} m/s")
    print(f"  speed |v|        : {speed:.3f} m/s")


if __name__ == "__main__":
    main()
