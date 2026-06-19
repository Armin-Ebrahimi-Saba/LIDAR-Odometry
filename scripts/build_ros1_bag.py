#!/usr/bin/env python3
"""Convert the Sensys ROS2 (db3) bag into a ROS1 bag that LIO_SAM_6AXIS can play.

LIO_SAM_6AXIS is a ROS1 package and expects:
  * a LiDAR PointCloud2 with per-point ``ring`` (uint16) and ``time`` (float32),
  * a ``sensor_msgs/Imu`` stream (6-axis: accel + gyro).

The Sensys bag provides neither directly:
  * ``/ouster/points`` is unorganized and has fields x,y,z,intensity,nearir,
    timeoffset -- no ``ring``. We synthesize ``ring`` from the elevation angle
    and map ``timeoffset`` (ms within the sweep) -> ``time`` (s, sweep-relative).
  * ``/ouster/imu_meas`` is the custom ``aspn_msgs/msg/MeasurementIMU`` which has
    no message definition available, so it cannot be deserialized normally. Its
    132-byte little-endian CDR payload is decoded by fixed byte offsets (verified
    against gravity): header stamp sec=int32@4 / nsec=uint32@8, specific force
    (m/s^2) = 3xfloat64 @68, angular rate (rad/s) = 3xfloat64 @92.

Output topics: ``/points_raw`` and ``/imu_raw`` (configurable below), written in
chronological (bag-time) order so ``rosbag play --clock`` replays them correctly.

Usage:
    python scripts/build_ros1_bag.py --config config.yaml
    python scripts/build_ros1_bag.py --config config.yaml --verify
    python scripts/build_ros1_bag.py --config config.yaml --max-scans 60   # smoke
"""
import argparse
import struct
from pathlib import Path

import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

# --- aspn_msgs/MeasurementIMU CDR byte offsets (little-endian, verified) -------
IMU_STAMP_SEC_OFF = 4      # int32
IMU_STAMP_NSEC_OFF = 8     # uint32
IMU_ACCEL_OFF = 68         # 3 x float64, specific force [m/s^2]
IMU_GYRO_OFF = 92          # 3 x float64, angular rate [rad/s]

# --- synthesized LiDAR geometry -----------------------------------------------
N_SCAN = 32                # Ouster beam count (detected from elevation clusters)

# Source /ouster/points layout (point_step = 24)
SRC_X_OFF, SRC_Y_OFF, SRC_Z_OFF = 0, 4, 8
SRC_INTENSITY_OFF = 12
SRC_TIMEOFFSET_OFF = 20    # float32, milliseconds within sweep

# Destination Velodyne PointXYZIRT layout (PCL EIGEN_ALIGN16, point_step = 32):
#   x@0 y@4 z@8 (pad@12) intensity@16 ring@20(uint16) (pad) time@24(float32)
DST_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "ring", "time"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
    "offsets": [0, 4, 8, 16, 20, 24],
    "itemsize": 32,
})
FLOAT32 = 7
UINT16 = 4

POINTS_OUT_TOPIC = "/points_raw"
IMU_OUT_TOPIC = "/imu_raw"
OUT_FRAME = "base_link"


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def decode_imu(raw):
    """Return (stamp_sec, stamp_nsec, accel(3,), gyro(3,)) from a MeasurementIMU CDR."""
    sec = struct.unpack_from("<i", raw, IMU_STAMP_SEC_OFF)[0]
    nsec = struct.unpack_from("<I", raw, IMU_STAMP_NSEC_OFF)[0]
    accel = np.array(struct.unpack_from("<ddd", raw, IMU_ACCEL_OFF))
    gyro = np.array(struct.unpack_from("<ddd", raw, IMU_GYRO_OFF))
    return sec, nsec, accel, gyro


def parse_points(msg):
    """Return finite x,y,z,intensity,timeoffset_ms arrays from a source PointCloud2.

    The source cloud is ``is_dense == False`` (contains NaN/invalid returns).
    LIO-SAM requires dense clouds and shuts down on NaN, so invalid points are
    dropped here and the output is emitted dense.
    """
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    x = arr[:, SRC_X_OFF:SRC_X_OFF + 4].copy().view("<f4").ravel()
    y = arr[:, SRC_Y_OFF:SRC_Y_OFF + 4].copy().view("<f4").ravel()
    z = arr[:, SRC_Z_OFF:SRC_Z_OFF + 4].copy().view("<f4").ravel()
    inten = arr[:, SRC_INTENSITY_OFF:SRC_INTENSITY_OFF + 4].copy().view("<f4").ravel()
    toff = arr[:, SRC_TIMEOFFSET_OFF:SRC_TIMEOFFSET_OFF + 4].copy().view("<f4").ravel()
    rng2 = x * x + y * y + z * z
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (rng2 > 0.0)
    return x[valid], y[valid], z[valid], inten[valid], toff[valid]


def estimate_elevation_bounds(reader, pc_conn, t0, t1, n_frames=40):
    """Robust [lo, hi] elevation (deg) over the first n_frames, for ring binning."""
    els = []
    seen = 0
    for conn, t, raw in reader.messages(connections=[pc_conn]):
        ts = t * 1e-9
        if ts < t0:
            continue
        if ts > t1:
            break
        msg = reader.deserialize(raw, conn.msgtype)
        x, y, z, _, _ = parse_points(msg)
        rng = np.sqrt(x * x + y * y + z * z)
        good = rng > 1.0
        el = np.degrees(np.arctan2(z[good], np.hypot(x[good], y[good])))
        els.append(el)
        seen += 1
        if seen >= n_frames:
            break
    el = np.concatenate(els)
    lo, hi = np.percentile(el, [0.5, 99.5])
    return float(lo), float(hi)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--verify", action="store_true",
                    help="Decode/inspect a few messages and report stats; write nothing.")
    ap.add_argument("--max-scans", type=int, default=0,
                    help="Stop after this many LiDAR scans (0 = all). For smoke tests.")
    ap.add_argument("--out", default=None, help="Output .bag path (default: <output_dir>/test1_lio.bag)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    bag_dir = cfg["paths"]["bag_dir"]
    lidar_topic = cfg["run"]["lidar_topic"]
    imu_topic = "/ouster/imu_meas"
    t0 = float(cfg["run"]["start_time"])
    t1 = float(cfg["run"]["end_time"])
    out_dir = Path(cfg["paths"]["output_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)
    out_bag = Path(args.out) if args.out else out_dir / "test1_lio.bag"

    ros2_ts = get_typestore(Stores.LATEST)

    with AnyReader([Path(bag_dir)], default_typestore=ros2_ts) as reader:
        pc_conn = next(c for c in reader.connections if c.topic == lidar_topic)
        imu_conn = next(c for c in reader.connections if c.topic == imu_topic)

        if args.verify:
            _verify(reader, pc_conn, imu_conn, t0, t1)
            return

        el_lo, el_hi = estimate_elevation_bounds(reader, pc_conn, t0, t1)
        print(f"[bag] elevation bounds for ring binning: [{el_lo:.2f}, {el_hi:.2f}] deg over {N_SCAN} beams")

        ros1_ts = get_typestore(Stores.ROS1_NOETIC)
        PointCloud2 = ros1_ts.types["sensor_msgs/msg/PointCloud2"]
        PointField = ros1_ts.types["sensor_msgs/msg/PointField"]
        Header = ros1_ts.types["std_msgs/msg/Header"]
        Time = ros1_ts.types["builtin_interfaces/msg/Time"]
        Imu = ros1_ts.types["sensor_msgs/msg/Imu"]
        Vector3 = ros1_ts.types["geometry_msgs/msg/Vector3"]
        Quaternion = ros1_ts.types["geometry_msgs/msg/Quaternion"]

        fields = [
            PointField(name="x", offset=0, datatype=FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=FLOAT32, count=1),
            PointField(name="intensity", offset=16, datatype=FLOAT32, count=1),
            PointField(name="ring", offset=20, datatype=UINT16, count=1),
            PointField(name="time", offset=24, datatype=FLOAT32, count=1),
        ]

        def make_header(sec, nsec):
            return Header(seq=0, stamp=Time(sec=int(sec), nanosec=int(nsec)), frame_id=OUT_FRAME)

        out_bag.parent.mkdir(parents=True, exist_ok=True)
        if out_bag.exists():
            out_bag.unlink()

        n_pc = n_imu = 0
        with Writer(out_bag) as writer:
            pc_w = writer.add_connection(POINTS_OUT_TOPIC, PointCloud2.__msgtype__, typestore=ros1_ts)
            imu_w = writer.add_connection(IMU_OUT_TOPIC, Imu.__msgtype__, typestore=ros1_ts)

            inv_span = (N_SCAN - 1) / (el_hi - el_lo)
            for conn, t, raw in reader.messages(connections=[pc_conn, imu_conn]):
                ts = t * 1e-9
                if ts < t0:
                    continue
                if ts > t1:
                    break

                if conn.topic == imu_topic:
                    sec, nsec, accel, gyro = decode_imu(raw)
                    msg = Imu(
                        header=make_header(sec, nsec),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                        orientation_covariance=np.array([-1.0] + [0.0] * 8),
                        angular_velocity=Vector3(x=gyro[0], y=gyro[1], z=gyro[2]),
                        angular_velocity_covariance=np.zeros(9),
                        linear_acceleration=Vector3(x=accel[0], y=accel[1], z=accel[2]),
                        linear_acceleration_covariance=np.zeros(9),
                    )
                    writer.write(imu_w, t, ros1_ts.serialize_ros1(msg, Imu.__msgtype__))
                    n_imu += 1
                else:
                    msg = reader.deserialize(raw, conn.msgtype)
                    x, y, z, inten, toff = parse_points(msg)
                    el = np.degrees(np.arctan2(z, np.hypot(x, y)))
                    ring = np.clip(np.round((el - el_lo) * inv_span), 0, N_SCAN - 1).astype("<u2")
                    out = np.zeros(x.shape[0], dtype=DST_DTYPE)
                    out["x"] = x
                    out["y"] = y
                    out["z"] = z
                    out["intensity"] = inten
                    out["ring"] = ring
                    out["time"] = (toff * 1e-3).astype("<f4")  # ms -> s, sweep-relative
                    data = np.frombuffer(out.tobytes(), dtype=np.uint8)
                    sec, nsec = divmod(t, 1_000_000_000)
                    pc = PointCloud2(
                        header=make_header(sec, nsec),
                        height=1,
                        width=out.shape[0],
                        fields=fields,
                        is_bigendian=False,
                        point_step=DST_DTYPE.itemsize,
                        row_step=DST_DTYPE.itemsize * out.shape[0],
                        data=data,
                        is_dense=True,
                    )
                    writer.write(pc_w, t, ros1_ts.serialize_ros1(pc, PointCloud2.__msgtype__))
                    n_pc += 1
                    if args.max_scans and n_pc >= args.max_scans:
                        break

        print(f"[bag] wrote {n_pc} clouds + {n_imu} imu msgs -> {out_bag}")
        print(f"[bag] size: {out_bag.stat().st_size / 1e9:.2f} GB")


def _verify(reader, pc_conn, imu_conn, t0, t1):
    print("== VERIFY: IMU decode ==")
    accs, gyrs = [], []
    n = 0
    for conn, t, raw in reader.messages(connections=[imu_conn]):
        ts = t * 1e-9
        if ts < t0:
            continue
        if ts > t1:
            break
        sec, nsec, accel, gyro = decode_imu(raw)
        accs.append(accel)
        gyrs.append(gyro)
        if n < 3:
            print(f"  stamp={sec}.{nsec:09d} bagt={ts:.3f} |a|={np.linalg.norm(accel):.3f} "
                  f"a={np.round(accel,3)} g={np.round(gyro,4)}")
        n += 1
    accs = np.array(accs)
    gyrs = np.array(gyrs)
    print(f"  msgs={n}  mean|a|={np.linalg.norm(accs,axis=1).mean():.3f}  "
          f"accel mean={np.round(accs.mean(0),3)}  gyro |mean|={np.round(np.abs(gyrs.mean(0)),4)}")

    print("== VERIFY: LiDAR ring synthesis ==")
    el_lo, el_hi = estimate_elevation_bounds(reader, pc_conn, t0, t1)
    print(f"  elevation bounds [{el_lo:.2f}, {el_hi:.2f}] deg, N_SCAN={N_SCAN}")
    seen = 0
    for conn, t, raw in reader.messages(connections=[pc_conn]):
        ts = t * 1e-9
        if ts < t0:
            continue
        if ts > t1:
            break
        msg = reader.deserialize(raw, conn.msgtype)
        x, y, z, inten, toff = parse_points(msg)
        el = np.degrees(np.arctan2(z, np.hypot(x, y)))
        inv_span = (N_SCAN - 1) / (el_hi - el_lo)
        ring = np.clip(np.round((el - el_lo) * inv_span), 0, N_SCAN - 1).astype(int)
        print(f"  cloud npts={x.shape[0]} ring[min,max]=[{ring.min()},{ring.max()}] "
              f"unique_rings={len(np.unique(ring))} time[ms]=[{toff.min():.1f},{toff.max():.1f}]")
        seen += 1
        if seen >= 3:
            break


if __name__ == "__main__":
    main()
