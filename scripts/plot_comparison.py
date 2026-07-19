import os
import glob
import math
import csv
import numpy as np
import json

a = 6378137.0
b = 6356752.314245
e2 = 1.0 - (b * b) / (a * a)

def LLA2ECEF(lat, lon, alt):
    lat = math.radians(lat)
    lon = math.radians(lon)
    N = a / math.sqrt(1.0 - e2 * math.sin(lat)**2)
    x = (N + alt) * math.cos(lat) * math.cos(lon)
    y = (N + alt) * math.cos(lat) * math.sin(lon)
    z = (N * (1.0 - e2) + alt) * math.sin(lat)
    return np.array([x, y, z])

def ECEF2ENU(x, y, z, lat0, lon0, alt0):
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)
    ecef0 = LLA2ECEF(lat0, lon0, alt0)
    dx, dy, dz = x - ecef0[0], y - ecef0[1], z - ecef0[2]
    
    sin_lat = math.sin(lat0_rad)
    cos_lat = math.cos(lat0_rad)
    sin_lon = math.sin(lon0_rad)
    cos_lon = math.cos(lon0_rad)
    
    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return np.array([e, n, u])

def ENU2ECEF(e, n, u, lat0, lon0, alt0):
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)
    ecef0 = LLA2ECEF(lat0, lon0, alt0)
    
    sin_lat = math.sin(lat0_rad)
    cos_lat = math.cos(lat0_rad)
    sin_lon = math.sin(lon0_rad)
    cos_lon = math.cos(lon0_rad)
    
    dx = -sin_lon * e - sin_lat * cos_lon * n + cos_lat * cos_lon * u
    dy = cos_lon * e - sin_lat * sin_lon * n + cos_lat * sin_lon * u
    dz = cos_lat * n + sin_lat * u
    return np.array([ecef0[0] + dx, ecef0[1] + dy, ecef0[2] + dz])

def ECEF2LLA(x, y, z):
    p = math.sqrt(x**2 + y**2)
    lon = math.atan2(y, x)
    lat = math.atan2(z, p * (1.0 - e2))
    for _ in range(5):
        N = a / math.sqrt(1.0 - e2 * math.sin(lat)**2)
        alt = p / math.cos(lat) - N
        lat = math.atan2(z, p * (1.0 - e2 * (N / (N + alt))))
    return math.degrees(lat), math.degrees(lon), alt

def load_tum(filename):
    data = np.loadtxt(filename)
    return data[:, 0], data[:, 1], data[:, 2] # t, x, y

def main():
    # 1. Load LIO-SAM without GNSS to get timestamps
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    odom_file = os.path.join(base_dir, 'output', 'maps', 'garden_day', 'optimized_odom_tum.txt')
    odom_lat, odom_lon = [], []
    odom_t, odom_x, odom_y = [], [], []
    if os.path.exists(odom_file):
        odom_t, odom_x, odom_y = load_tum(odom_file)

    # 2. Load LIO-SAM with GNSS
    gnss_folders = [f for f in glob.glob(os.path.join(base_dir, 'output', 'maps_gnss*')) if os.path.isdir(f)]
    gnss_folders.sort()
    odom_gnss_lat, odom_gnss_lon = [], []
    odom_gnss_t, odom_gnss_x, odom_gnss_y = [], [], []
    if gnss_folders:
        latest_gnss_folder = gnss_folders[-1]
        odom_gnss_file = os.path.join(latest_gnss_folder, 'garden_day', 'optimized_odom_tum.txt')
        if os.path.exists(odom_gnss_file):
            odom_gnss_t, odom_gnss_x, odom_gnss_y = load_tum(odom_gnss_file)

    min_t = float('inf')
    max_t = float('-inf')
    if len(odom_t) > 0:
        min_t = min(min_t, odom_t[0])
        max_t = max(max_t, odom_t[-1])
    if len(odom_gnss_t) > 0:
        min_t = min(min_t, odom_gnss_t[0])
        max_t = max(max_t, odom_gnss_t[-1])

    # 3. Load Ground Truth and Filter
    gt_file = os.path.join(base_dir, 'data', 'ground_truth_gnss', 'xtrack_global_position_t12.csv')
    with open(gt_file, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    
    if min_t < float('inf'):
        filtered_rows = []
        for r in rows:
            t = float(r['timestamp'])
            # Pad by 20 seconds to be safe
            if min_t - 20 <= t <= max_t + 20:
                filtered_rows.append(r)
        if len(filtered_rows) > 0:
            rows = filtered_rows
            print(f"Filtered Ground Truth to {len(rows)} points based on LIO-SAM time range.")
    
    import json
    gnss_origin_file = os.path.join(base_dir, 'data', 'gnss_origin.json')
    origin_lat, origin_lon, origin_alt = None, None, None
    prevYaw = 0.0
    if os.path.exists(gnss_origin_file):
        with open(gnss_origin_file, 'r') as f:
            origin_data = json.load(f)
            origin_lat = origin_data['lat']
            origin_lon = origin_data['lon']
            origin_alt = origin_data['alt']
            prevYaw = origin_data['prevYaw']
            print(f"Loaded origin from GNSS: {origin_lat}, {origin_lon}, yaw: {prevYaw}")

    if origin_lat is None:
        origin_lat = float(rows[0]['lat'])
        origin_lon = float(rows[0]['lon'])
        origin_alt = float(rows[0]['alt'])
        
        gt_enu = []
        for row in rows:
            ecef = LLA2ECEF(float(row['lat']), float(row['lon']), float(row['alt']))
            enu = ECEF2ENU(ecef[0], ecef[1], ecef[2], origin_lat, origin_lon, origin_alt)
            gt_enu.append(enu)
        gt_enu = np.array(gt_enu)

        for enu in gt_enu:
            distance = math.sqrt(enu[0]**2 + enu[1]**2)
            if distance > 0.1:
                prevYaw = math.atan2(enu[1], enu[0])
                break
    else:
        gt_enu = []
        for row in rows:
            ecef = LLA2ECEF(float(row['lat']), float(row['lon']), float(row['alt']))
            enu = ECEF2ENU(ecef[0], ecef[1], ecef[2], origin_lat, origin_lon, origin_alt)
            gt_enu.append(enu)
        gt_enu = np.array(gt_enu)

    gt_lat = []
    gt_lon = []
    for row in rows:
        gt_lat.append(float(row['lat']))
        gt_lon.append(float(row['lon']))

    # 4. Process LIO-SAM coordinates
    if len(odom_x) > 0:
        # First, apply the prevYaw as LIO-SAM did internally
        cos_yaw = math.cos(prevYaw)
        sin_yaw = math.sin(prevYaw)
        odom_e = []
        odom_n = []
        for x, y in zip(odom_x, odom_y):
            e = x * cos_yaw - y * sin_yaw
            n = x * sin_yaw + y * cos_yaw
            odom_e.append(e)
            odom_n.append(n)
            
        yaw_diff = 0.0
        # Now, calculate a yaw correction to align the initial direction!
        # We use raw_gnss.json because the Ground Truth has a weird initialization loop.
        raw_gnss_file = os.path.join(base_dir, 'data', 'raw_gnss.json')
        if os.path.exists(raw_gnss_file) and len(odom_e) > 0:
            with open(raw_gnss_file, 'r') as f:
                raw_gnss_data = json.load(f)
            
            raw_enu = []
            for d in raw_gnss_data:
                ecef = LLA2ECEF(d['lat'], d['lon'], origin_alt)
                e, n, u = ECEF2ENU(ecef[0], ecef[1], ecef[2], origin_lat, origin_lon, origin_alt)
                raw_enu.append((e, n))

            def get_dir_vec(e_list, n_list, target_dist=60.0):
                for e, n in zip(e_list, n_list):
                    if math.hypot(e - e_list[0], n - n_list[0]) > target_dist:
                        return e - e_list[0], n - n_list[0]
                return e_list[-1] - e_list[0], n_list[-1] - n_list[0]
                
            vec_odom = get_dir_vec(odom_e, odom_n, target_dist=60.0)
            vec_raw = get_dir_vec([p[0] for p in raw_enu], [p[1] for p in raw_enu], target_dist=60.0)
            
            angle_odom = math.atan2(vec_odom[1], vec_odom[0])
            angle_raw = math.atan2(vec_raw[1], vec_raw[0])
            yaw_diff = angle_raw - angle_odom
            
            print(f"Applying Yaw Correction (60m interval on raw GNSS): {math.degrees(yaw_diff):.2f} degrees")
            
            cos_diff = math.cos(yaw_diff)
            sin_diff = math.sin(yaw_diff)
            
            for i in range(len(odom_e)):
                e_corr = odom_e[i] * cos_diff - odom_n[i] * sin_diff
                n_corr = odom_e[i] * sin_diff + odom_n[i] * cos_diff
                ecef = ENU2ECEF(e_corr, n_corr, 0.0, origin_lat, origin_lon, origin_alt)
                lat, lon, _ = ECEF2LLA(ecef[0], ecef[1], ecef[2])
                odom_lat.append(lat)
                odom_lon.append(lon)
        else:
            for i in range(len(odom_e)):
                ecef = ENU2ECEF(odom_e[i], odom_n[i], 0.0, origin_lat, origin_lon, origin_alt)
                lat, lon, _ = ECEF2LLA(ecef[0], ecef[1], ecef[2])
                odom_lat.append(lat)
                odom_lon.append(lon)

    if len(odom_gnss_x) > 0:
        # We need to use the EXACT prevYaw that was used when generating the BLUE line.
        # gnss_origin.json was just updated to the new 5.0m prevYaw (-2.249).
        cos_yaw = math.cos(prevYaw)
        sin_yaw = math.sin(prevYaw)
        
        for x, y in zip(odom_gnss_x, odom_gnss_y):
            # 1. Apply internal prevYaw rotation (this reverts the rotation LIO-SAM added)
            # Since mapOptimization used GPS factors rotated by -prevYaw, applying +prevYaw
            # perfectly restores the true real-world ENU coordinates!
            # We DO NOT apply yaw_diff here!
            e = x * cos_yaw - y * sin_yaw
            n = x * sin_yaw + y * cos_yaw
            
            ecef = ENU2ECEF(e, n, 0.0, origin_lat, origin_lon, origin_alt)
            lat, lon, _ = ECEF2LLA(ecef[0], ecef[1], ecef[2])
            odom_gnss_lat.append(lat)
            odom_gnss_lon.append(lon)

    # 4. Dump to JSON and write HTML
    data = {
        'gt_lat': gt_lat,
        'gt_lon': gt_lon,
        'odom_lat': odom_lat,
        'odom_lon': odom_lon,
        'odom_gnss_lat': odom_gnss_lat,
        'odom_gnss_lon': odom_gnss_lon
    }
    
    raw_gnss_js = ""
    if os.path.exists(raw_gnss_file):
        with open(raw_gnss_file, 'r') as f:
            raw_gnss_data = json.load(f)
        raw_gnss_js = f"""
        const raw_gnss = JSON.parse('{json.dumps(raw_gnss_data)}');
        traces.push({{
            type: 'scattermapbox',
            lat: raw_gnss.map(d => d.lat),
            lon: raw_gnss.map(d => d.lon),
            mode: 'lines',
            name: 'Raw Built-in GNSS',
            line: {{color: 'black', width: 2, dash: 'dash'}}
        }});
        """

    html_content = f"""<!DOCTYPE html>
<html>
<head>
    <title>Trajectory Map Comparison</title>
    <script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
</head>
<body style="margin:0; padding:0;">
    <div id="plot" style="width:100vw;height:100vh;"></div>
    <script>
        const data = {json.dumps(data)};
        
        const traces = [];
        
        traces.push({{
            type: 'scattermapbox',
            lat: data.gt_lat,
            lon: data.gt_lon,
            mode: 'lines',
            name: 'Ground Truth (GNSS)',
            line: {{color: 'green', width: 4}}
        }});
        
        {raw_gnss_js}
        if (data.odom_lat.length > 0) {{
            traces.push({{
                type: 'scattermapbox',
                lat: data.odom_lat,
                lon: data.odom_lon,
                mode: 'lines',
                name: 'LIO-SAM (No GNSS)',
                line: {{color: 'red', width: 3}}
            }});
        }}
        
        if (data.odom_gnss_lat.length > 0) {{
            traces.push({{
                type: 'scattermapbox',
                lat: data.odom_gnss_lat,
                lon: data.odom_gnss_lon,
                mode: 'lines',
                name: 'LIO-SAM (With GNSS)',
                line: {{color: 'blue', width: 3}}
            }});
        }}
        
        const layout = {{
            title: 'Trajectory Comparison on OpenStreetMap',
            mapbox: {{
                style: 'open-street-map',
                center: {{lat: data.gt_lat[0], lon: data.gt_lon[0]}},
                zoom: 18
            }},
            margin: {{r: 0, t: 40, b: 0, l: 0}},
            showlegend: true
        }};
        
        Plotly.newPlot('plot', traces, layout);
    </script>
</body>
</html>
"""
    html_output = os.path.join(base_dir, 'output', 'plot_viewer_map.html')
    with open(html_output, 'w', encoding='utf-8') as f:
        f.write(html_content)
        
    print("Saved map plot to plot_viewer_map.html. Please open this file in your browser!")

if __name__ == '__main__':
    main()
