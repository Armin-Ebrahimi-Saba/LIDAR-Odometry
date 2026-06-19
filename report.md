# Sensys Data — Complete Inventory Report

* **Generated:** 2026-06-14  
* **Server Location:** `/home/armin/Experiments/sensys/data/`  
* **Total Dataset Size:** ~36 GB  
* **Target Area:** Berlin, Germany (~52.515°N, 13.323°E)

---

## 1. Dataset Overview & Sensor Suite
This folder contains sensor data from two autonomous test runs (**Test1** and **Test2**) conducted via an **XTrack vehicle platform**.

```
 ┌────────────────────────────────── XTrack Vehicle Platform ──────────────────────────────────┐
 │                                                                                             │
 │  [PX4 Autopilot]         [Ouster LiDAR]            [Intel RealSense D435i]   [GNSS System]  │
 │  GPS, Odometry,          3D Point Clouds,          Stereo RGB-D Color        RTK-Corrected  │
 │  Attitude, Local Pos     IMU (Acc/Gyro + Quat)     Images & Depth Maps       Ground Truth   │
 └─────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Core Test Runs (ROS 2 Bags)

Both datasets are stored as **ROS 2 Bagfiles (v5, SQLite3)** inside compressed `.zip` files. They share an identical internal directory structure and identical ROS 2 topics.

### 2.1 Metadata Comparison

| Run Metric | Test Run 1 (`Test1_data.zip`) | Test Run 2 (`Test2_data.zip`) |
| :--- | :--- | :--- |
| **Compressed Size** | 21 GB (20 GiB) | 16 GB (15 GiB) |
| **Duration** | 822 seconds (~13 min 42 sec) | 602 seconds (~10 min 2 sec) |
| **Time (UTC)** | 2026-06-02 13:32 → 13:46 | 2026-06-02 13:45 → 13:55 |
| **Total Messages** | 442,072 | 323,346 |
| **LiDAR Clouds (`.laz`)**| 8,194 files | 6,008 files |

### 2.2 ROS 2 Topic Manifest & Frequencies

| Sensor Subsystem | Topic Name | Frequency | Test 1 Count | Test 2 Count | Data Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **LiDAR Points** | `/ouster/points` | ~10 Hz | 8,194 | 6,008 | 3D Point Clouds (`sensor_msgs/PointCloud2`) |
| **LiDAR IMU** | `/ouster/imu_meas`<br>`/ouster/imu_att`<br>`/ouster/status` | ~100 Hz<br>~100 Hz<br>~10 Hz | 81,945<br>81,944<br>8,194 | 59,877<br>59,961<br>6,007 | Raw Accel/Gyro metrics<br>3D Attitude Quaternions<br>Health diagnostics |
| **PX4 Autopilot** | `/fmu/out/vehicle_odometry`<br>`/fmu/out/vehicle_attitude`<br>`/fmu/out/vehicle_local_position_v1`<br>`/fmu/out/vehicle_gps_position` | ~100 Hz<br>~100 Hz<br>~50 Hz<br>~10 Hz | 81,386<br>81,799<br>41,105<br>8,223 | 59,559<br>59,835<br>30,085<br>6,019 | Pose & Velocity tracking<br>Attitude Quaternions<br>Local NED positions<br>Raw GPS readings |
| **RGB-D Camera** | `/camera/d435i/color/image_raw`<br>`/camera/d435i/color/camera_info`<br>`/camera/d435i/depth/image_rect_raw`<br>`/camera/d435i/depth/camera_info` | ~15 Hz<br>~15 Hz<br>~15 Hz<br>~15 Hz | 12,319<br>12,320<br>12,320<br>12,322 | 9,006<br>9,006<br>8,991<br>8,990 | Standard RGB frames<br>Color intrinsics matrices<br>Rectified stereo depth map<br>Depth intrinsics matrices |
| **Static Transforms**| `/tf_static` | Once | 1 | 1 | Sensor frame extrinsics |

### 2.3 Internal Zip Structure
```
[TestX_data.zip]
└── rosbag/
    ├── rosbag_0.db3      # Primary SQLite3 Database (~16-18 GB unzipped)
    ├── metadata.yaml     # Topic manifest & ROS 2 metadata configuration
    └── rosbag_data/
        └── laz_clouds/
            └── cloud_############.laz  # Compressed individual point clouds
```

---

## 3. Post-Processed GNSS Ground Truth (`xtrack_gnss_corrected/`)

This folder contains high-accuracy global position data used as the absolute **reference trajectory** for error analysis.

### 3.1 Data File Breakdown

#### 📊 `xtrack_global_position_t12.csv` (~600 KB | 7,686 rows)
* **Purpose:** Highly accurate post-processed reference trajectory (SF6/RTK corrected). Use this for RMSE and drift evaluation.
* **Key Columns:** `timestamp`, `lat`/`lon` (WGS84), `alt` (MSL), `alt_ellipsoid`, `eph`/`epv` (Accuracy), and validity flags (`lat_lon_valid`, `alt_valid`).

#### 📊 `xtrack_gps_position_t12.csv` (~2 MB | 15,369 rows)
* **Purpose:** Raw PX4 GPS data coupled with comprehensive health/quality metadata.
* **Key Columns:** `fix_type` (2=DGPS, 3=RTK Float, 4=RTK Fix), `satellites_used`, `hdop`/`vdop`, `jamming_indicator`, and NED velocities (`vel_n_m_s`, `vel_e_m_s`, `vel_d_m_s`).

#### 🗺️ Visualization Files (Non-Source Data)
* **`xtrack_globalpos_vs_gps.kml`:** Google Earth trajectory line strings (**Red:** Corrected global track | **Green:** Raw drifting GPS).
* **`xtrack_globalpos_vs_gps_satellite_map.html`:** Interactive Leaflet / OpenStreetMap web application tracking path variances over satellite imagery.

---

## 4. Pipeline Reference Architecture

```
                                ┌───────────────────────────────────┐
                                │     Test1 & Test2 Rosbag Data     │
                                └─────────────────┬─────────────────┘
                                                  │
         ┌────────────────────────────────────────┼────────────────────────────────────────┐
         ▼                                        ▼                                        ▼
┌───────────────────────────────┐        ┌───────────────────────────────┐        ┌───────────────────────────────┐
│     LiDAR SLAM Pipeline       │        │   Visual Inertial Odometry    │        │    Person Tracking System     │
├───────────────────────────────┤        ├───────────────────────────────┤        ├───────────────────────────────┤
│ Input:                        │        │ Input:                        │        │ Input:                        │
│ • `.laz` Point Clouds         │        │ • Color Images (`/color`)     │        │ • Color Frames (Aruco Vests)  │
│ • IMU Data (`/ouster/imu_as`) │        │ • Camera Intrinsics (`_info`) │        │ • Depth Maps (`/depth`)       │
│                               │        │ • IMU Data (`/ouster/imu_as`) │        │ • Static Transforms (`/tf`)   │
├───────────────────────────────┤        ├───────────────────────────────┤        ├───────────────────────────────┤
│ Output Target:                │        │ Output Target:                │        │ Output Target:                │
│ • 2D Trajectory (CSV)         │        │ • 2D Velocity & Paths (CSV)   │        │ • Relative Target Locations   │
│ • 3D Combined Map (`.pcd`)    │        │ • Optical Drift Analysis      │        │ • Target Trajectories         │
└───────────────────────┬───────┘        └───────────────┬───────────────┘        └───────────────┬───────────────┘
                        │                                │                                │
                        └────────────────────────────────┼────────────────────────────────┘
                                                         ▼
                                      ┌───────────────────────────────────┐
                                      │    Validation & Error Metrics     │
                                      ├───────────────────────────────────┤
                                      │ Baseline Reference Sources:       │
                                      │ • `xtrack_global_position_t12.csv`│
                                      │ • `xtrack_gps_position_t12.csv`   │
                                      └───────────────────────────────────┘
```

---

## 5. Coordinate & Time Configuration

### 5.1 System Frames
* **Global Positions:** WGS84 Geographic Coordinate System (Latitude, Longitude, Altitude).
* **PX4 Autopilot Local Frame:** **NED** (North-East-Down) spatial body frame baseline centered at vehicle takeoff location.
* **LiDAR Points:** Ouster sensor coordinate layout (typically mapped to ENU or NED specs).
* **Camera Pixels:** Standard optical coordinates originating from top-left array.
* **ROS Transform Tree:** Use `/tf_static` to calculate structural offsets, and `vehicle_odometry` to compute body-to-world state conversions.

### 5.2 Key Timestamp Windows (Unix Epoch Seconds)
* **Test Run 1:** `1780397390.972` — `1780398213.329` (822s window)
* **Test Run 2:** `1780398327.532` — `1780398929.389` (602s window)
* **GNSS Reference Ground Truth:** `1780397225.802` — `1780398777.808` (Includes extended pre/post lock ranges)

---

## 6. Utilities & Limitations

### 6.1 Data Format Quick Commands
* **`.zip` Archives:** Decompress files using `unzip Test1_data.zip -d /target/dir/`
* **`.db3` ROS 2 Bags:** Evaluate profiles using `ros2 bag info <path>` or index via standard `sqlite3` CLI wrappers.
* **`.laz` Point Clouds:** Process natively with `laspy` (Python package via `laszip` support) or compile with standard `pdal` conversion tools.
* **`.csv` Layouts:** Ingest files instantly via Python: `pandas.read_csv('filename.csv')`

### 6.2 Known Pipeline Bottlenecks & Gaps
* **IMU Realignment Required:** No standalone frame-mounted IMU exists. The system relies entirely on the Ouster-integrated tracker. Compensate for frame disparities using the `/tf_static` transforms vector.
* **No External Validation Trackers:** Subjects / tracking targets did not wear remote loggers. Compute coordinate estimation directly through LiDAR & Camera data fusion.
* **Extraction Processing Costs:** Point clouds are compressed (`.laz`) and video assets are bound to SQLite tables. Target bags must be unpacked and passed to extraction scripts (`cv_bridge` or `rosbag dump`) before training pipelines run.

---

## 7. Recommended Quickstart Implementation Order

1. **Unpack Primary Workspace:** Extract `Test1_data.zip` first. It provides the largest sample frame dataset for preliminary tuning (14 minutes of continuous tracking data).
2. **Establish Ground Truth Base:** Link `xtrack_global_position_t12.csv` as your absolute spatial baseline index for error optimization computations.
3. **Configure Noise Rejection Filters:** Use `xtrack_gps_position_t12.csv` to strip unreliable frames. **Drop any coordinates showing an active `fix_type < 4` or an `eph > 1.0`.**
