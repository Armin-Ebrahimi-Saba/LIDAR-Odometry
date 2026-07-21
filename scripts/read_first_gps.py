#!/usr/bin/env python3
"""Read and print the first /fmu/out/vehicle_gps_position frame from the bag.

The bag stores data/px4_msgs/msg/SensorGps as raw CDR but does NOT embed the message
definition (see metadata.yaml), so we register it here from the upstream
px4_msgs SensorGps.msg field layout. This bag's version predates the trailing
`antenna_offset_{x,y,z}` fields (they are absent from the exported
xtrack_gps_position_t12.csv), so they are omitted to keep the CDR layout exact.

Usage:
    python scripts/read_first_gps.py [BAG_DIR]

Default BAG_DIR is ./data/rosbag.
"""
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

GPS_TOPIC = "/fmu/out/vehicle_gps_position"

# data/px4_msgs/msg/SensorGps -- field order/types must match the recorded message
# exactly for CDR decoding. This mirrors upstream SensorGps.msg minus the
# trailing antenna_offset_{x,y,z} fields, which this bag's version does not have.
_SENSOR_GPS_MSG = """
uint64 timestamp
uint64 timestamp_sample
uint32 device_id
float64 latitude_deg
float64 longitude_deg
float64 altitude_msl_m
float64 altitude_ellipsoid_m
float32 s_variance_m_s
float32 c_variance_rad
uint8 fix_type
float32 eph
float32 epv
float32 hdop
float32 vdop
int32 noise_per_ms
uint16 automatic_gain_control
uint8 jamming_state
int32 jamming_indicator
uint8 spoofing_state
uint8 authentication_state
float32 vel_m_s
float32 vel_n_m_s
float32 vel_e_m_s
float32 vel_d_m_s
float32 cog_rad
bool vel_ned_valid
int32 timestamp_time_relative
uint64 time_utc_usec
uint8 satellites_used
uint32 system_error
float32 heading
float32 heading_offset
float32 heading_accuracy
float32 rtcm_injection_rate
uint8 selected_rtcm_instance
bool rtcm_crc_failed
uint8 rtcm_msg_used
"""


def _typestore_with_sensor_gps():
    ts = get_typestore(Stores.LATEST)
    ts.register(get_types_from_msg(_SENSOR_GPS_MSG, "data/px4_msgs/msg/SensorGps"))
    return ts


def read_first_gps(bag_dir: str):
    """Return the first SensorGps message on GPS_TOPIC and its bag-record time (s)."""
    ts = _typestore_with_sensor_gps()
    with AnyReader([Path(bag_dir)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == GPS_TOPIC]
        if not conns:
            available = sorted({c.topic for c in reader.connections})
            raise SystemExit(
                f"Topic '{GPS_TOPIC}' not found in {bag_dir}.\n"
                "Available topics:\n  " + "\n  ".join(available)
            )
        for conn, t_ns, raw in reader.messages(connections=conns):
            return reader.deserialize(raw, conn.msgtype), t_ns * 1e-9
    raise SystemExit(f"No messages on '{GPS_TOPIC}'.")


def main():
    bag_dir = sys.argv[1] if len(sys.argv) > 1 else "./data/rosbag"
    msg, bag_time_s = read_first_gps(bag_dir)

    print(f"First {GPS_TOPIC} frame from {bag_dir}")
    print(f"  bag-record time : {bag_time_s:.6f} s (Unix epoch)")
    print( "  --- message fields ---")
    print(f"  timestamp           : {msg.timestamp} us  ({msg.timestamp * 1e-6:.6f} s)")
    print(f"  timestamp_sample    : {msg.timestamp_sample} us")
    print(f"  device_id           : {msg.device_id}")
    print(f"  latitude_deg        : {msg.latitude_deg:.9f}")
    print(f"  longitude_deg       : {msg.longitude_deg:.9f}")
    print(f"  altitude_msl_m      : {msg.altitude_msl_m:.4f}")
    print(f"  altitude_ellipsoid_m: {msg.altitude_ellipsoid_m:.4f}")
    print(f"  fix_type            : {msg.fix_type}")
    print(f"  eph / epv           : {msg.eph:.3f} / {msg.epv:.3f}")
    print(f"  hdop / vdop         : {msg.hdop:.3f} / {msg.vdop:.3f}")
    print(f"  vel N/E/D (m/s)     : {msg.vel_n_m_s:.3f} / {msg.vel_e_m_s:.3f} / {msg.vel_d_m_s:.3f}")
    print(f"  cog_rad             : {msg.cog_rad:.4f}")
    print(f"  satellites_used     : {msg.satellites_used}")
    print(f"  time_utc_usec       : {msg.time_utc_usec}")
    print(f"  heading             : {msg.heading}")


if __name__ == "__main__":
    main()
