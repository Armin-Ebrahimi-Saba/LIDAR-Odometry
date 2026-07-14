#!/usr/bin/env python3
"""Pure-inertial (strapdown) speed of the platform over a LiDAR frame range.

Unlike `imu_speed.py` -- which reports the PX4 EKF velocity (IMU-propagated but
GPS-corrected) -- this script uses **only** the raw Ouster IMU (`/ouster/imu_meas`,
accel + gyro) and mechanizes it by hand: integrate the gyro for orientation,
rotate the accelerometer into a level nav frame, subtract gravity, and integrate
the residual acceleration into velocity. No GPS, no EKF, no external aiding.

This is "speed from the IMU" in the literal sense, and it drifts accordingly:
with no correction, accelerometer bias and (mostly) gyro-bias-driven gravity
leakage make the velocity run away over tens of seconds. Use `--validate-static
A B` to quantify that drift -- run the exact same mechanization over a segment
where the platform is known to be still, and whatever speed comes out is pure
error. On Test1 the platform is stationary for the first ~1000 frames, which is
what makes an initial velocity of zero (`--v0-zero`, the default) defensible.

The bag stores `/ouster/imu_meas` as raw CDR (aspn_msgs/MeasurementIMU) with no
embedded definition; the layout is fixed and decoded directly: builtin Time at
byte 0, then a length-prefixed sensor-id string, then six float64 at byte 64 =
accel[3] (m/s^2) followed by gyro[3] (rad/s).

Usage:
    python scripts/imu_pure_speed.py 1000 1500
    python scripts/imu_pure_speed.py 1000 1500 --validate-static 200 700
    python scripts/imu_pure_speed.py 1000 1500 --calib 100 900 --plot out.png
"""
import argparse
import struct
from pathlib import Path

import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation as Rot

IMU_TOPIC = "/ouster/imu_meas"
# Fixed CDR layout of aspn_msgs/MeasurementIMU in this bag (see module docstring).
_TIME_OFFSET = 0            # <II  : sec, nsec (time_of_validity, == bag time)
_IMU6_OFFSET = 64           # <6d  : accel_x,y,z (m/s^2), gyro_x,y,z (rad/s)


def read_imu(bag_dir: str):
    """Return (t[s], accel[N,3] m/s^2, gyro[N,3] rad/s) from /ouster/imu_meas."""
    ts = get_typestore(Stores.LATEST)
    t, acc, gyr = [], [], []
    with AnyReader([Path(bag_dir)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == IMU_TOPIC]
        if not conns:
            available = sorted({c.topic for c in reader.connections})
            raise SystemExit(
                f"Topic '{IMU_TOPIC}' not found in {bag_dir}.\n"
                "Available topics:\n  " + "\n  ".join(available))
        for _c, _t_ns, raw in reader.messages(connections=conns):
            body = bytes(raw)[4:]                       # strip CDR encapsulation
            sec, nsec = struct.unpack_from("<II", body, _TIME_OFFSET)
            six = struct.unpack_from("<6d", body, _IMU6_OFFSET)
            t.append(sec + nsec * 1e-9)
            acc.append(six[:3])
            gyr.append(six[3:])
    return np.asarray(t), np.asarray(acc), np.asarray(gyr)


def lidar_frame_times(bag_dir: str, topic: str) -> np.ndarray:
    """Bag-record times (s) of every message on `topic`, in order."""
    ts = get_typestore(Stores.LATEST)
    times = []
    with AnyReader([Path(bag_dir)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise SystemExit(f"Topic '{topic}' not found in {bag_dir}.")
        for _c, t_ns, _raw in reader.messages(connections=conns):
            times.append(t_ns * 1e-9)
    return np.asarray(times)


def calibrate(imu_t, acc, gyr, t0, t1):
    """From a static window [t0,t1]: gravity vector (body frame) and gyro bias.

    While the platform is still, the mean accelerometer reading is the gravity
    reaction (its direction levels the IMU) and the mean gyro is pure bias.
    """
    m = (imu_t >= t0) & (imu_t <= t1)
    if m.sum() < 2:
        raise SystemExit("Empty calibration window; check --calib frames.")
    return acc[m].mean(0), gyr[m].mean(0)


def strapdown_speed(imu_t, acc, gyr, t0, t1, gravity_body, gyro_bias,
                    v0=np.zeros(3)):
    """Integrate raw IMU over [t0,t1] -> (times, speeds).

    Nav frame is anchored to the IMU body frame at t0. Orientation is propagated
    from the gyro (bias removed); the accelerometer is rotated into that frame
    and the constant gravity vector is subtracted before integrating to velocity.
    """
    m = (imu_t >= t0) & (imu_t <= t1)
    tt = imu_t[m]
    a = acc[m]
    g = gyr[m] - gyro_bias
    R = np.eye(3)
    v = np.asarray(v0, float).copy()
    times = [tt[0]]
    speeds = [np.linalg.norm(v)]
    for k in range(1, len(tt)):
        dt = tt[k] - tt[k - 1]
        R = R @ Rot.from_rotvec(g[k - 1] * dt).as_matrix()
        a_nav = R @ a[k - 1] - gravity_body        # remove gravity reaction
        v = v + a_nav * dt
        times.append(tt[k])
        speeds.append(np.linalg.norm(v))
    return np.asarray(times), np.asarray(speeds)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frame_start", type=int, help="first LiDAR frame of the window")
    ap.add_argument("frame_end", type=int, help="last LiDAR frame of the window")
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--calib", nargs=2, type=int, metavar=("A", "B"), default=[100, 900],
                    help="static frame range for gravity/gyro-bias calibration "
                         "(default: 100 900, the Test1 stationary start)")
    ap.add_argument("--validate-static", nargs=2, type=int, metavar=("A", "B"),
                    default=None, help="run the mechanization over a known-static "
                    "frame range and report the resulting speed as pure drift")
    ap.add_argument("--no-gyro-debias", action="store_true",
                    help="do not remove the static gyro bias (shows raw drift)")
    ap.add_argument("--plot", default=None, help="write a speed-vs-time PNG here")
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    bag_dir = cfg["paths"]["bag_dir"]
    topic = cfg["run"].get("lidar_topic", "/ouster/points")

    pts_t = lidar_frame_times(bag_dir, topic)
    imu_t, acc, gyr = read_imu(bag_dir)
    n = len(pts_t)
    for f in (args.frame_start, args.frame_end, *args.calib):
        if f < 0 or f >= n:
            raise SystemExit(f"Frame {f} out of range (topic has {n} frames).")

    gravity_body, gyro_bias = calibrate(
        imu_t, acc, gyr, pts_t[args.calib[0]], pts_t[args.calib[1]])
    if args.no_gyro_debias:
        gyro_bias = np.zeros(3)

    print(f"Pure-IMU (strapdown) speed  --  raw {IMU_TOPIC}, no GPS/EKF aiding")
    print(f"  calibration window : frames {args.calib[0]}..{args.calib[1]}  "
          f"({pts_t[args.calib[1]] - pts_t[args.calib[0]]:.1f} s static)")
    print(f"  gravity |g|        : {np.linalg.norm(gravity_body):.3f} m/s^2")
    print(f"  gyro bias          : {np.degrees(gyro_bias).round(3)} deg/s"
          f"{'  (NOT removed)' if args.no_gyro_debias else ''}")

    if args.validate_static:
        a, b = args.validate_static
        _, sv = strapdown_speed(imu_t, acc, gyr, pts_t[a], pts_t[b],
                                gravity_body, gyro_bias)
        print(f"\n  VALIDATION on static frames {a}..{b} (true speed = 0):")
        print(f"    pure-IMU drift -> mean {sv.mean():.3f}, max {sv.max():.3f}, "
              f"final {sv[-1]:.3f} m/s  (this much is pure error)")

    t0, t1 = pts_t[args.frame_start], pts_t[args.frame_end]
    times, speeds = strapdown_speed(imu_t, acc, gyr, t0, t1,
                                    gravity_body, gyro_bias)
    dur = t1 - t0
    print(f"\n  FRAMES {args.frame_start}..{args.frame_end}  ({dur:.1f} s, "
          f"v0 = 0 assumed at frame {args.frame_start}):")
    print(f"    speed -> mean {speeds.mean():.3f}, max {speeds.max():.3f}, "
          f"final {speeds[-1]:.3f} m/s")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot(times - t0, speeds, label="pure-IMU |v|")
        ax.set_xlabel("time since frame %d [s]" % args.frame_start)
        ax.set_ylabel("speed [m/s]")
        ax.set_title("Pure-IMU strapdown speed, frames %d..%d"
                     % (args.frame_start, args.frame_end))
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        print(f"\n  wrote {args.plot}")


if __name__ == "__main__":
    main()
