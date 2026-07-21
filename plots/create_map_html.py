import pandas as pd
import numpy as np
import json

def latlon_to_local_xy(lats, lons):
    """Convert an array of Lat/Lon (which was actually projected at equator) to local X/Y"""
    R = 6378137.0
    lat0 = np.radians(lats[0])
    lon0 = np.radians(lons[0])
    
    lats_rad = np.radians(lats)
    lons_rad = np.radians(lons)
    
    y = (lats_rad - lat0) * R
    x = (lons_rad - lon0) * R * np.cos(lat0)
    
    return x, y

def local_xy_to_latlon(x, y, lat0_deg, lon0_deg):
    """Convert local X/Y to real geographic Lat/Lon in Berlin"""
    R = 6378137.0
    lat0_rad = np.radians(lat0_deg)
    
    delta_lat = np.degrees(y / R)
    delta_lon = np.degrees(x / (R * np.cos(lat0_rad)))
    
    return lat0_deg + delta_lat, lon0_deg + delta_lon

def align_around_start(source_x, source_y, target_x, target_y, align_fraction=0.1):
    """
    Translates source so its start point matches target's start point,
    then rotates source around the start point to minimize overall distance.
    Resamples to the same length first so the mean is comparable.
    """
    s = np.linspace(0, 1, len(source_x))
    t = np.linspace(0, 1, len(target_x))
    tgt_x_resampled = np.interp(s, t, target_x)
    tgt_y_resampled = np.interp(s, t, target_y)
    
    src = np.vstack((source_x, source_y)).T
    tgt = np.vstack((tgt_x_resampled, tgt_y_resampled)).T
    
    # Translate to start point (0,0)
    src_centered = src - src[0]
    tgt_centered = tgt - tgt[0]
    
    N = max(2, int(len(src) * align_fraction))
    U, _, Vt = np.linalg.svd(np.dot(tgt_centered[:N].T, src_centered[:N]))
    R = np.dot(U, Vt)
    
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = np.dot(U, Vt)
        
    src_aligned = np.dot(src_centered, R.T) + tgt[0]
    return src_aligned[:, 0], src_aligned[:, 1]



def main():
    print("Loading data...")
    # Load LIO-SAM data first to find max timestamp
    slam_wo_df = pd.read_csv('output/LIO_SAM_maps_wo_gnss/SLAM_path.csv').iloc[::2, :]
    slam_w_df = pd.read_csv('output/LIO_SAM_maps_with_gnss/SLAM_path.csv').iloc[::2, :]
    
    max_slam_t = max(slam_wo_df['timestamp'].max(), slam_w_df['timestamp'].max())
    TIME_OFFSET = 313.75
    
    # Load Ground Truth
    gt_df = pd.read_csv('output/ground_truth_gnss/xtrack_gps_position_t12.csv')
    
    # Crop GT to only the relevant time window (remove the extra track)
    # Give a small 10s buffer
    gt_df = gt_df[gt_df['timestamp'] <= (max_slam_t - TIME_OFFSET + 10)]
    
    gt_df = gt_df.iloc[::2, :] # Subsample for performance in browser
    gt_lat = gt_df['latitude_deg'].values
    gt_lon = gt_df['longitude_deg'].values
    
    # Origin of GT
    origin_lat = gt_lat[0]
    origin_lon = gt_lon[0]

    # To do Procrustes, we first need GT in local coordinates
    gt_x, gt_y = latlon_to_local_xy(gt_lat, gt_lon)

    # Convert SLAM without GNSS
    slam_wo_x, slam_wo_y = latlon_to_local_xy(slam_wo_df['lat'].values, slam_wo_df['lon'].values)
    
    # Align to GT shape to fix heading drift (rotate around start point)
    slam_wo_x_aligned, slam_wo_y_aligned = align_around_start(slam_wo_x, slam_wo_y, gt_x, gt_y)
    slam_wo_lat, slam_wo_lon = local_xy_to_latlon(slam_wo_x_aligned, slam_wo_y_aligned, origin_lat, origin_lon)

    # Convert SLAM with GNSS
    slam_w_x, slam_w_y = latlon_to_local_xy(slam_w_df['lat'].values, slam_w_df['lon'].values)
    
    # Do NOT rotate GNSS path. Only translate it (which is already done since latlon_to_local_xy sets both to 0,0)
    slam_w_lat, slam_w_lon = local_xy_to_latlon(slam_w_x, slam_w_y, origin_lat, origin_lon)

    print("Generating HTML...")
    html_content = f"""<!DOCTYPE html>
<html>
<head>
    <title>LIO-SAM vs GNSS Ground Truth</title>
    <script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
</head>
<body style="margin:0; padding:0;">
    <div id="plot" style="width:100vw;height:100vh;"></div>
    <script>
        const gt_lat = {json.dumps(list(gt_lat))};
        const gt_lon = {json.dumps(list(gt_lon))};
        
        const slam_wo_lat = {json.dumps(list(slam_wo_lat))};
        const slam_wo_lon = {json.dumps(list(slam_wo_lon))};
        
        const slam_w_lat = {json.dumps(list(slam_w_lat))};
        const slam_w_lon = {json.dumps(list(slam_w_lon))};

        const traceGT = {{
            type: 'scattermapbox',
            mode: 'lines',
            name: 'Ground Truth (GNSS)',
            lat: gt_lat,
            lon: gt_lon,
            line: {{color: 'black', width: 4}}
        }};

        const traceSLAMwo = {{
            type: 'scattermapbox',
            mode: 'lines',
            name: 'LIO-SAM (w/o GNSS)',
            lat: slam_wo_lat,
            lon: slam_wo_lon,
            line: {{color: 'red', width: 3}}
        }};

        const traceSLAMw = {{
            type: 'scattermapbox',
            mode: 'lines',
            name: 'LIO-SAM (w/ GNSS)',
            lat: slam_w_lat,
            lon: slam_w_lon,
            line: {{color: 'blue', width: 3}}
        }};

        const layout = {{
            mapbox: {{
                style: "open-street-map",
                center: {{ lat: {origin_lat}, lon: {origin_lon} }},
                zoom: 16
            }},
            margin: {{ r: 0, t: 0, b: 0, l: 0 }},
            showlegend: true,
            legend: {{
                x: 0.02,
                y: 0.98,
                bgcolor: 'rgba(255,255,255,0.8)',
                font: {{size: 16}}
            }}
        }};

        Plotly.newPlot('plot', [traceGT, traceSLAMwo, traceSLAMw], layout);
    </script>
</body>
</html>
"""

    with open('plots/plot_viewer_map2.html', 'w') as f:
        f.write(html_content)
    print("Saved to plots/plot_viewer_map2.html")

if __name__ == "__main__":
    main()
