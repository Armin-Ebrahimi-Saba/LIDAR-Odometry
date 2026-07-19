import sqlite3
from rosbags.rosbag2 import Reader


def main():
    with Reader('rosbag_new') as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/fmu/out/vehicle_gps_position':
                print(f"Rawdata len: {len(rawdata)}")
                print(f"Header: {rawdata[:4].hex()}")
                print(f"Next 4 bytes: {rawdata[4:8].hex()}")
                print(f"Next 8 bytes (timestamp): {rawdata[8:16].hex()}")
                import struct
                ts, = struct.unpack('<Q', rawdata[8:16])
                print(f"Timestamp: {ts}")
                
                # Let's try to unpack assuming 4 byte padding
                ts, ts_sample, dev_id, lat, lon, alt = struct.unpack('<QQIiii', rawdata[8:40])
                print(f"With 8:40 -> lat: {lat/1e7}, lon: {lon/1e7}")
                
                # Let's try 4:36
                try:
                    ts, ts_sample, dev_id, lat, lon, alt = struct.unpack('<QQIiii', rawdata[4:36])
                    print(f"With 4:36 -> lat: {lat/1e7}, lon: {lon/1e7}")
                except Exception as e:
                    print("Error 4:36", e)
                break

if __name__ == '__main__':
    main()
