import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

def latlon_to_local_xy(lats, lons):
    """
    Convert an array of Lat/Lon to local X/Y coordinates in meters,
    relative to the first point in the array.
    """
    R = 6378137.0 # WGS-84 Earth radius in meters
    lat0 = np.radians(lats[0])
    lon0 = np.radians(lons[0])
    
    lats_rad = np.radians(lats)
    lons_rad = np.radians(lons)
    
    y = (lats_rad - lat0) * R
    x = (lons_rad - lon0) * R * np.cos(lat0)
    
    return x, y

def align_around_start(source_x, source_y, target_x, target_y, align_fraction=0.1):
    """
    Translates source so its start point matches target's start point,
    then rotates source around the start point to minimize overall distance
    along the first `align_fraction` segment of the trajectory.
    """
    src = np.vstack((source_x, source_y)).T
    tgt = np.vstack((target_x, target_y)).T
    
    # Translate to start point (0,0)
    src_centered = src - src[0]
    tgt_centered = tgt - tgt[0]
    
    # SVD to find optimal rotation around origin using only the beginning
    N = max(2, int(len(src) * align_fraction))
    U, _, Vt = np.linalg.svd(np.dot(tgt_centered[:N].T, src_centered[:N]))
    R = np.dot(U, Vt)
    
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = np.dot(U, Vt)
        
    src_aligned = np.dot(src_centered, R.T) + tgt[0]
    return src_aligned[:, 0], src_aligned[:, 1]


def process_and_plot(gt_csv, slam_csvs, labels, out_file):
    # Load Ground Truth
    print(f"Loading Ground Truth: {gt_csv}")
    gt_df = pd.read_csv(gt_csv)
    # The GT file has columns: timestamp, latitude_deg, longitude_deg
    gt_times = gt_df['timestamp'].values
    gt_lats = gt_df['latitude_deg'].values
    gt_lons = gt_df['longitude_deg'].values
    
    # The ROSBAG and GNSS Ground Truth timestamps are NOT synchronized!
    # They have an offset of ~313.7 seconds.
    # We apply this offset to ground truth so it matches the SLAM timestamps.
    TIME_OFFSET = 313.75
    gt_times_synced = gt_times + TIME_OFFSET
    
    plt.figure(figsize=(12, 6))
    
    for slam_csv, label in zip(slam_csvs, labels):
        if not os.path.exists(slam_csv):
            print(f"Warning: File {slam_csv} not found, skipping...")
            continue
            
        print(f"Loading SLAM Data: {slam_csv} ({label})")
        slam_df = pd.read_csv(slam_csv)
        slam_times = slam_df['timestamp'].values
        slam_lats = slam_df['lat'].values
        slam_lons = slam_df['lon'].values
        
        # Interpolate GT positions at the exact SLAM timestamps using SYNCHRONIZED times
        matched_gt_lats = np.interp(slam_times, gt_times_synced, gt_lats, left=np.nan, right=np.nan)
        matched_gt_lons = np.interp(slam_times, gt_times_synced, gt_lons, left=np.nan, right=np.nan)
        
        # Filter out points where GT data doesn't exist
        valid_mask = ~np.isnan(matched_gt_lats) & ~np.isnan(matched_gt_lons)
        
        if not np.any(valid_mask):
            print(f"  -> No overlapping timestamps with Ground Truth for {label}!")
            continue
            
        valid_slam_times = slam_times[valid_mask]
        valid_slam_lats = slam_lats[valid_mask]
        valid_slam_lons = slam_lons[valid_mask]
        
        valid_gt_lats = matched_gt_lats[valid_mask]
        valid_gt_lons = matched_gt_lons[valid_mask]
        
        # Convert to local X/Y (meters)
        # Note: latlon_to_local_xy automatically translates the start point to (0,0)
        slam_x, slam_y = latlon_to_local_xy(valid_slam_lats, valid_slam_lons)
        gt_x, gt_y = latlon_to_local_xy(valid_gt_lats, valid_gt_lons)
        
        # Only align "w/o GNSS" (rotate around start to match initial segment). 
        # "w/ GNSS" uses its raw GNSS heading.
        if "o" in label.lower() or "ohne" in label.lower():
            slam_x_aligned, slam_y_aligned = align_around_start(slam_x, slam_y, gt_x, gt_y)
        else:
            slam_x_aligned, slam_y_aligned = slam_x, slam_y
        
        # Calculate instantaneous Position Error (in meters)
        errors_m = np.sqrt((slam_x_aligned - gt_x)**2 + (slam_y_aligned - gt_y)**2)
        
        # Calculate RMSE
        rmse = np.sqrt(np.mean(errors_m**2))
        print(f"  -> Total RMSE for {label}: {rmse:.3f} meters")
        
        # Plot relative to the first overlapping timestamp
        relative_times = valid_slam_times - valid_slam_times[0]
        
        plt.plot(relative_times, errors_m, label=f"{label} (RMSE: {rmse:.2f}m)", linewidth=2, alpha=0.8)

    plt.title("Position Error (RMSE) over Time", fontsize=16, fontweight='bold', pad=15)
    plt.xlabel("Time (seconds)", fontsize=12)
    plt.ylabel("Position Error (meters)", fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(fontsize=12)
    
    # Save the plot
    print(f"\nSaving plot to {out_file}...")
    plt.tight_layout()
    plt.savefig(out_file, dpi=300, facecolor='white')

def main():
    parser = argparse.ArgumentParser(description="Plot RMSE over time for LIO-SAM against GNSS Ground Truth.")
    parser.add_argument('--gt', type=str, required=True, help="Path to ground truth CSV (e.g., xtrack_gps_position_t12.csv)")
    parser.add_argument('--slam', nargs='+', type=str, required=True, help="Paths to the SLAM_path.csv files")
    parser.add_argument('--labels', nargs='+', type=str, required=True, help="Labels for the SLAM runs (must match --slam count)")
    parser.add_argument('--out', type=str, default="plots/rmse_comparison.png", help="Output image filename")
    
    args = parser.parse_args()
    
    if len(args.slam) != len(args.labels):
        print("Error: The number of --slam files must match the number of --labels")
        return
        
    process_and_plot(args.gt, args.slam, args.labels, args.out)

if __name__ == '__main__':
    main()
