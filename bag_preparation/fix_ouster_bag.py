import numpy as np
from pathlib import Path
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import get_typestore, Stores
import sys

def main():
    inbag_path = Path('bags/rosbag_new.bag')
    outbag_path = Path('bags/lio_sam_ready.bag')

    if not inbag_path.exists():
        print(f"Error: {inbag_path} not found.")
        sys.exit(1)

    print(f"Reading from {inbag_path}")
    print(f"Writing to {outbag_path}")

    # Typestore für ROS 1 initialisieren
    store = get_typestore(Stores.ROS1_NOETIC)
    PointField = store.types['sensor_msgs/msg/PointField']

    # Datentypen für Numpy definieren
    # Input: x, y, z, intensity, nearir, timeoffset (alle FLOAT32)
    dt_in = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
        ('nearir', np.float32),
        ('timeoffset', np.float32)
    ])

    # Output: x, y, z, intensity (FLOAT32), t (FLOAT32), ring (UINT16)
    # LIO-SAM ist meist tolerant beim Datentyp von 't', solange der Name stimmt.
    dt_out = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
        ('t', np.uint32), 
        ('ring', np.uint16)
    ])

    min_angle = -0.785398 # -45 Grad in Bogenmaß
    max_angle = 0.785398  # +45 Grad in Bogenmaß
    angle_range = max_angle - min_angle

    with Reader(inbag_path) as reader, Writer(outbag_path) as writer:
        conn_map = {}
        for conn in reader.connections:
            # Erstelle die Verbindungen im neuen Bag
            conn_map[conn.id] = writer.add_connection(
                conn.topic, 
                conn.msgtype, 
                typestore=store
            )

        count_points = 0
        count_other = 0

        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/ouster/points':
                # Deserialisiere PointCloud2
                msg = store.deserialize_ros1(rawdata, connection.msgtype)
                
                # Numpy Array aus den binären Daten erstellen
                points = np.frombuffer(msg.data, dtype=dt_in)
                
                # Vertikalen Winkel (Pitch) berechnen: arctan2(z, sqrt(x^2 + y^2))
                xy_dist = np.sqrt(points['x']**2 + points['y']**2)
                pitch = np.arctan2(points['z'], xy_dist)
                
                # Winkel auf 0 bis 31 mappen (32 Ringe)
                normalized = (pitch - min_angle) / angle_range
                ring_floats = np.nan_to_num(normalized * 31.0, nan=0.0)
                rings = np.round(ring_floats).astype(np.uint16)
                rings = np.clip(rings, 0, 31) # Sicherheitshalber auf [0, 31] clippen

                # Neues Array mit dem Ziel-Format erstellen
                out_points = np.zeros(len(points), dtype=dt_out)
                out_points['x'] = points['x']
                out_points['y'] = points['y']
                out_points['z'] = points['z']
                out_points['intensity'] = points['intensity']
                out_points['t'] = (points['timeoffset'] * 1e6).astype(np.uint32) # Umbenennen für LIO-SAM und in Nanosekunden konvertieren
                out_points['ring'] = rings
                
                # PointCloud2 Nachricht updaten
                msg.data = np.frombuffer(out_points.tobytes(), dtype=np.uint8)
                msg.row_step = len(out_points) * dt_out.itemsize
                msg.point_step = dt_out.itemsize
                
                # Neue Felder definieren
                msg.fields = [
                    PointField(name='x', offset=0, datatype=7, count=1), # FLOAT32 = 7
                    PointField(name='y', offset=4, datatype=7, count=1),
                    PointField(name='z', offset=8, datatype=7, count=1),
                    PointField(name='intensity', offset=12, datatype=7, count=1),
                    PointField(name='t', offset=16, datatype=6, count=1), # UINT32 = 6
                    PointField(name='ring', offset=20, datatype=4, count=1) # UINT16 = 4
                ]
                
                # Wieder serialisieren und speichern
                out_rawdata = store.serialize_ros1(msg, connection.msgtype)
                writer.write(conn_map[connection.id], timestamp, out_rawdata)
                count_points += 1
            else:
                # Andere Nachrichten (wie IMU) einfach 1:1 kopieren
                writer.write(conn_map[connection.id], timestamp, rawdata)
                count_other += 1
                
            if (count_points + count_other) % 1000 == 0:
                print(f"Processed {count_points + count_other} messages...")

    print(f"Fertig! Punktwolken: {count_points}, Andere Nachrichten: {count_other}")
    print(f"Die Datei {outbag_path} ist nun bereit für LIO-SAM.")

if __name__ == '__main__':
    main()
