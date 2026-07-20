import os
import sys
import math
import argparse
import csv
import glob

def enu_to_latlon(x, y, z, lat0, lon0, alt0):
    """
    Convert local ENU coordinates to Latitude, Longitude, Altitude.
    Using flat-earth approximation.
    """
    R = 6378137.0 # WGS-84 Earth radius in meters
    lat0_rad = math.radians(lat0)
    
    delta_lat = math.degrees(y / R)
    delta_lon = math.degrees(x / (R * math.cos(lat0_rad)))
    
    lat = lat0 + delta_lat
    lon = lon0 + delta_lon
    alt = alt0 + z
    
    return lat, lon, alt

def main():
    parser = argparse.ArgumentParser(description="Format LIO-SAM output to match specified folder structure.")
    parser.add_argument('--map_dir', type=str, required=True, help='Path to the directory containing LIO-SAM outputs')
    
    args = parser.parse_args()
    map_dir = args.map_dir
    
    if not os.path.exists(map_dir):
        print(f"Error: Directory {map_dir} does not exist.")
        sys.exit(1)
        
    print(f"Processing directory: {map_dir}")
    
    # 1. Read Origin
    origin_file = os.path.join(map_dir, 'origin.txt')
    lat0, lon0, alt0 = 0.0, 0.0, 0.0
    if os.path.exists(origin_file):
        with open(origin_file, 'r') as f:
            line = f.readline().strip()
            if line:
                parts = line.split()
                if len(parts) >= 3:
                    lat0, lon0, alt0 = float(parts[0]), float(parts[1]), float(parts[2])
                    print(f"Found origin: Lat={lat0}, Lon={lon0}, Alt={alt0}")
    else:
        print("Warning: origin.txt not found. Using default origin (0, 0, 0).")
        
    # 2. Process Trajectory and compute Velocity
    odom_file = os.path.join(map_dir, 'optimized_odom_tum.txt')
    if not os.path.exists(odom_file):
        # Fallback to standard odom
        odom_file = os.path.join(map_dir, 'odom_tum.txt')
        
    csv_file = os.path.join(map_dir, 'SLAM_path.csv')
    
    if os.path.exists(odom_file):
        print(f"Reading odometry from {os.path.basename(odom_file)}...")
        data_rows = []
        with open(odom_file, 'r') as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 8:
                    ts = float(parts[0])
                    tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                    data_rows.append((ts, tx, ty, tz))
                    
        print(f"Loaded {len(data_rows)} poses. Calculating LatLon and NED velocity...")
        
        # Calculate LatLon and Velocity
        # TUM is ENU: x = East, y = North, z = Up
        # NED velocity: v_n = delta y, v_e = delta x, v_d = -delta z
        csv_data = []
        for i in range(len(data_rows)):
            ts, tx, ty, tz = data_rows[i]
            lat, lon, alt = enu_to_latlon(tx, ty, tz, lat0, lon0, alt0)
            
            if i == 0:
                vn, ve, vd = 0.0, 0.0, 0.0
            else:
                prev_ts, prev_tx, prev_ty, prev_tz = data_rows[i-1]
                dt = ts - prev_ts
                if dt > 0:
                    vn = (ty - prev_ty) / dt
                    ve = (tx - prev_tx) / dt
                    vd = -(tz - prev_tz) / dt # Down is negative Up
                else:
                    vn, ve, vd = 0.0, 0.0, 0.0
                    
            csv_data.append([f"{ts:.6f}", f"{lat:.8f}", f"{lon:.8f}", f"{vn:.6f}", f"{ve:.6f}", f"{vd:.6f}"])
            
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "lat", "lon", "v_n", "v_e", "v_d"])
            writer.writerows(csv_data)
        print(f"Saved {csv_file}")
    else:
        print("Warning: No TUM odometry file found. SLAM_path.csv will not be generated.")

    # 3. Rename PCD file
    pcd_src = os.path.join(map_dir, 'globalmap_lidar_feature.pcd')
    pcd_dst = os.path.join(map_dir, 'map3d.pcd')
    if os.path.exists(pcd_src):
        os.rename(pcd_src, pcd_dst)
        print(f"Renamed {os.path.basename(pcd_src)} to map3d.pcd")
    elif os.path.exists(pcd_dst):
        print("map3d.pcd already exists.")
    else:
        print("Warning: globalmap_lidar_feature.pcd not found.")

    # 4. Clean up other files
    print("Cleaning up old files...")
    patterns_to_remove = [
        "odom_tum.txt",
        "optimized_odom_tum.txt",
        "optimized_odom_kitti.txt",
        "optimized_gps_trajectry.kml",
        "pose_graph.g2o",
        "times.txt",
        "origin.txt",
        "*.bag",
        "*.bag.bak"
    ]
    
    removed_count = 0
    for pattern in patterns_to_remove:
        for f in glob.glob(os.path.join(map_dir, pattern)):
            if os.path.isfile(f):
                os.remove(f)
                removed_count += 1
                
    print(f"Removed {removed_count} old files.")
    print("Format and cleanup completed successfully.")

if __name__ == '__main__':
    main()
