from rosbags.rosbag2 import Reader
import struct

with Reader('./rosbag') as reader:
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/ouster/imu_meas':
            payload = rawdata[4:]
            print(f"Payload size: {len(payload)}")
            for i in range(0, len(payload), 8):
                chunk = payload[i:i+8]
                if len(chunk) == 8:
                    fval, = struct.unpack('<d', chunk)
                    ival, = struct.unpack('<q', chunk)
                    print(f"Offset {i:3d}: hex={chunk.hex()} float={fval:10.5g} int={ival}")
            break
