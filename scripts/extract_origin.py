import rosbag
import math
import json

def main():
    bag = rosbag.Bag('/workspace/bags/lio_sam_ready.bag')
    first_lla = None
    prevYaw = 0.0

    a = 6378137.0
    b = 6356752.314245
    e2 = 1.0 - (b * b) / (a * a)

    def LLA2ECEF(lat, lon, alt):
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        N = a / math.sqrt(1.0 - e2 * math.sin(lat_rad)**2)
        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1.0 - e2) + alt) * math.sin(lat_rad)
        return x, y, z

    def ECEF2ENU(x, y, z, lat0, lon0, alt0):
        lat0_rad = math.radians(lat0)
        lon0_rad = math.radians(lon0)
        ecef0_x, ecef0_y, ecef0_z = LLA2ECEF(lat0, lon0, alt0)
        dx = x - ecef0_x
        dy = y - ecef0_y
        dz = z - ecef0_z
        sin_lat = math.sin(lat0_rad)
        cos_lat = math.cos(lat0_rad)
        sin_lon = math.sin(lon0_rad)
        cos_lon = math.cos(lon0_rad)
        e = -sin_lon * dx + cos_lon * dy
        n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        return e, n, u

    raw_gps_data = []
    
    for topic, msg, t in bag.read_messages(topics=['/gps/fix']):
        if first_lla is None:
            first_lla = (msg.latitude, msg.longitude, msg.altitude)
        else:
            if prevYaw == 0.0:
                ecef_x, ecef_y, ecef_z = LLA2ECEF(msg.latitude, msg.longitude, msg.altitude)
                e, n, u = ECEF2ENU(ecef_x, ecef_y, ecef_z, first_lla[0], first_lla[1], first_lla[2])
                dist = math.sqrt(e**2 + n**2)
                if dist > 5.0:
                    prevYaw = math.atan2(n, e)
        raw_gps_data.append({'lat': msg.latitude, 'lon': msg.longitude})

    bag.close()
    
    with open('/workspace/data/gnss_origin.json', 'w') as f:
        json.dump({'lat': first_lla[0], 'lon': first_lla[1], 'alt': first_lla[2], 'prevYaw': prevYaw}, f)
        
    with open('/workspace/data/raw_gnss.json', 'w') as f:
        json.dump(raw_gps_data, f)
    
    print("Extracted origin: " + str(first_lla) + ", prevYaw: " + str(prevYaw))

if __name__ == '__main__':
    main()
