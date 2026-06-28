## Task 3 — LIDAR-based Positioning

- **Simple:** LIDAR Odometry based on Scan-Scan or Scan-Map matching.
- **Advanced:** SLAM — Simultaneous Localization and Mapping.

**Requirements:**

- 2D trajectory (LatLon) and velocity (NED-frame) estimation result (e.g. as `.csv` file).
- 3D point cloud map (e.g. as `.pcd` file); resolution can be minimized.
- Error plot: estimate vs ground truth (GNSS), using metrics like RMSE.

**Tip — use toolboxes available on GitHub:**

- Useful repositories from: https://github.com/koide3
- LIO-SAM: https://github.com/JokerJohn/LIO_SAM_6AXIS
- ORB-SLAM: https://github.com/UZ-SLAMLab/ORB_SLAM3
- KISS-ICP: https://github.com/prbonn/kiss-icp
- GenZ-ICP: https://github.com/93won/lidar_odometry

> **Note:** IMU attitude is often used for scan-deskewing.

---

## Sensors on XTrack

| Sensor | Details |
|--------|---------|
| **Ouster OS0** | 32 lines, 360° 3D LIDAR; 90° HFOV, < 100 m range; points at 10 Hz |
| **IMU (from Pixhawk)** | Integrated attitude solution at 100 Hz; mag-heading included |
| **Intel Realsense D435i** | RGB images 640×480; depth images 640×480; images at 10–15 Hz |
| **Jetson Orin NX** | Computing unit; Ubuntu 22.04 + ROS2 |
| **Ublox GNSS Receiver** | Serves as ground truth source; position in Lat/Lon |

---

## Coordinate Frames

- Define a local coordinate system on the XTrack.
- Visit manufacturers' websites for sensor frames:
  - Ouster LIDAR: https://static.ouster.dev/sensordocs/image_route1/image_route2/sensor_data/sensor-data.html
  - Realsense D435i: https://www.intelrealsense.com/depth-camera-d435i/
  - Realsense ROS: https://github.com/IntelRealSense/realsense-ros

### Coordinate Frames & Mounting

- **Idea:** Take the Pixhawk coordinate system as the body-frame.
  - X-axis pointing forward, Y-axis pointing right, Z-axis pointing down.
- Need more? Ask!

---

## Provided Data

**Alternative 1 — use raw data:**

- Images as `.png` files with UTC timestamps (ns) in the filename, e.g. `color_1780327043165317871`
- Depth images as `.png` with UTC timestamps (ns) in the filename
- Point clouds as `.laz`
- IMU and GNSS data as `.csv`

**Alternative 2 — ROS2 bag:**

- You need to install beforehand:
  - PX4 msgs (release/1.17): https://github.com/PX4/px4_msgs
  - `ros2 bag play <path to bag>`
- **Attention:** depending on your PC, playback and especially rendering (like RViz) of large bags with point clouds and images might cause delays; reduce rendering or try the provided FastDDS profile.

Link for data download provided on ISIS.

---

## Datasets

### Outdoor 1

- Outdoor driving with XTrack from ILR to Ernst-Reuter-Platz and back.
  - Straight movements, stops, and turns.
  - GNSS ground truth position from XTrack a bit noisy.

### Outdoor 2

- Outdoor driving with XTrack around the ILR / math building.
  - Less building structure.
  - Some noise in GNSS ground truth.
  - May be harder for navigation.

### Use of Datasets

- Two datasets are provided.
- You do not need to run your code for the full dataset if not possible.
- Feel free to select parts of one or both datasets.
- However, the longer the segment used to prove your algorithm, the better (and more challenging).
- **Suggestion:** start with dataset 1 and use as much of it as possible.

---

## GNSS Ground Truth

- Both datasets already include the GNSS position of the XTrack.
- But: this is the **raw GNSS solution** and is noisy.
- A filtered solution was extracted from the Pixhawk logs and provided as CSV:
  - `vehicle_global_position` (instead of `vehicle_gps_position`)
- **Recommendation:** use this as ground truth for the XTrack.
- Find it in the data folder: `SenSy26/xtrack_gnss_corrected/xtrack_global_position_t12.csv`
  - The `timestamp_sample` is the same as in the rosbags / provided CSV files.

> **Note:**
> - There is still some noise / jumps at the beginning and end.
> - One file covers both datasets — filter by timestamp.

---

## Using ROS2

- Optional, not required.
- But: may be a good chance to get familiar with it.
- **Recommendation:** ROS2 Humble, if possible.
  - If your system requires: ROS1 or another ROS2 distribution.
- Show data in ROS2:
  - `ros2 bag play <name>`

> For your convenience: ROS2 bag files for both datasets are uploaded.

---

## Requirements

**Final report** (max 10 pages):

- Code architecture (include diagrams)
- Choice of algorithms / system design
  - Refer to assumptions, simplifications, used parameters
- Results / evaluation (including challenges)
- Include graphs and images wherever possible

**Code database** (as `.zip` or GitHub project):

- Including a README with setup and use instructions

**1–3 min "Teaser" Video** (counts as presentation):

- Pitch your ideas (used algorithms, structural design)
- Show results (live execution, plots, maps) — keep it short (no need to show the whole test)
- With vocal or text explanations — someone with no idea about the topic should be able to follow
