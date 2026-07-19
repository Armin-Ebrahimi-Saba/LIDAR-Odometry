# Sensor System Report: Application of LIO-SAM 6AXIS
<video
src="https://github.com/user-attachments/assets/8927424c-57e8-4176-9076-d3b6719260ea"
loop autoplay muted controls></video>


## 1. Overview of Files and Scripts

The following scripts and configurations work together to perform the mapping:

*   **`docker-compose.yml` & `Dockerfile`**: 
    These build and start an isolated ROS Melodic container in which LIO-SAM 6AXIS is installed. The current directory is mounted as `/workspace` in the container to share scripts and rosbags.
*   **`fix_ouster_bag.py`**:
    *(Preparation)* A Python script that synchronizes and adjusts the timestamps of the IMU and LiDAR data in the original `rosbag`. Ouster sensors without PTP time synchronization often have issues with highly asynchronous timestamps, which causes SLAM algorithms to crash. The script outputs the repaired file `lio_sam_ready.bag`.
*   **`patch_yaml.py`**:
    *(Configuration)* This script is executed automatically when the container starts. It modifies the LIO-SAM configuration file `indoor_ouster128.yaml` directly in the container. It makes essential changes:
    *   **Extrinsic Rotation & RPY**: Set to the identity matrix (`[1,0,0, 0,1,0, 0,0,1]`), as the Ouster IMU is not physically rotated.
    *   **Gravity (`imuGravity`)**: Set to `-9.80511`, as the Ouster IMU measures the actual gravity downwards (in contrast to the ROS REP-145 convention, which specifies the reaction force upwards).
    *   **IMU Noise/Bias (`imuAccNoise`, etc.)**: Increased, as the internal IMU is relatively inaccurate. Without this adjustment, the optimization would trust the IMU sensor too much and drift (diverge) after a few minutes due to accumulated errors.
*   **`run_lio_sam.sh`**:
    *(Execution)* The main script that is started inside the Docker container. It performs the following tasks:
    1.  Injects `use_sim_time` into the LIO-SAM launch file so that LIO-SAM uses the timestamps of the rosbag instead of the real computer clock.
    2.  Starts LIO-SAM and RViz in the background.
    3.  Starts the playback of the `lio_sam_ready.bag` and remaps the topic names to those expected by LIO-SAM (`/ouster/points` -> `/os_cloud_node/points`, `/ouster/imu_meas` -> `/stim300/imu/data_raw`).
    4.  Automatically executes a save operation (`rosservice call ...`) as soon as the rosbag has finished playing.

---

## 2. Step-by-Step Guide

### Step 1: Prepare Rosbag (Only the first time or for new data)
Since raw Ouster data often has problematic timestamps, the rosbag must be cleaned up first. Ensure that your raw rosbag file is located in the folder (e.g., `rosbag2_2024_...`).
Run the script on your host PC (with the `rosbags` Python package installed):
```bash
python fix_ouster_bag.py <your_input_bag.bag> lio_sam_ready.bag
```
*This script has already been executed and `lio_sam_ready.bag` is ready.*

### Step 2: Start Docker Container
Make sure Docker Desktop is running. Open a terminal (PowerShell) in this folder and run:
```bash
docker compose up
```
*(To run it in the background, use `docker compose up -d`, but without `-d` you will see all outputs and error messages directly).*

### Step 3: The Automatic Process
As soon as `docker compose up` is running, the following happens automatically:
1. The container executes `run_lio_sam.sh`.
2. The script patches the configuration (`patch_yaml.py`).
3. LIO-SAM and RViz are started. RViz opens on your computer via X11 (VcXsrv).
4. The `lio_sam_ready.bag` is played. You will see the map slowly building up in RViz.

### Step 4: Save Results
Once the rosbag has completely finished (after approx. 13-14 minutes), the following message is displayed in the terminal:
`Rosbag finished playing. Automatically saving the map to /workspace/maps/...`

The script then calls the ROS Service `/lio_sam_6axis/save_map`. 
**You will find the saved `.pcd` point cloud files directly on your Windows desktop in the new folder `maps/`.**

> **Manual Saving:** If you want to abort the process early and save the current state, open a second terminal in this folder, attach to the running container and call the service manually:
> `docker exec lio_sam_6axis /bin/bash -c "source /opt/ros/melodic/setup.bash && rosservice call /lio_sam_6axis/save_map"`

---

## 3. Troubleshooting

*   **RViz does not open:** Ensure that VcXsrv (Xming) is running on Windows and `Disable access control` is enabled in the settings.
*   **"Large velocity" warnings:** If these errors appear in the logs and the map explodes, the IMU configuration is incorrect. Check if `patch_yaml.py` was called correctly.
*   **Map remains empty in RViz:** In RViz, click "Add" on the left -> "PointCloud2" and set the topic to `/lio_sam_6axis/mapping/map_global` or `/lio_sam_6axis/deskew/cloud_deskewed`.

## Control bag play
* Pause: `docker exec lio_sam_6axis /bin/bash -c "source /opt/ros/melodic/setup.bash && rosservice call /bag_player/pause_playback true"`

* Continue: `docker exec lio_sam_6axis /bin/bash -c "source /opt/ros/melodic/setup.bash && rosservice call /bag_player/pause_playback false"`

---

## 4. Advanced GNSS (GPS) Integration

To ensure the accuracy of the map over long distances and to compensate for LiDAR drift, the system was expanded with advanced GNSS integration. In our case, the standard LIO-SAM implementation had issues with the highly noisy GPS at the starting point (spaghetti node).

These issues were resolved with the following **C++ patches** in LIO-SAM:
*   **`simpleGpsOdom_patched.cpp`**: The odometry node calculation now waits until the robot (based purely on LiDAR odometry) has moved **5 meters** away from the starting point. Only then is a stable, reliable alignment (Yaw) calculated from the covered distance and the origin for the GNSS is set. This prevents faulty starting angles caused by stationary sensor noise.
*   **`mapOptmizationGps.cpp`**: Instead of waiting for the GPS signal to be initialized (which would now take 5 meters and block the entire SLAM process), the LiDAR mapping process starts immediately and independently. As soon as the first reliable GPS coordinates arrive after 5 meters, LIO-SAM retroactively inserts them as optimization factors (GTSAM graph) and seamlessly aligns the map created so far with the GNSS.

> **Note:** Since `mapOptmizationGps.cpp` was modified, this file is now mounted directly as a volume in the container via `docker-compose.yml` so that it is compiled live when the container starts (via `catkin build`).

### 4.1. Start LIO-SAM with GNSS
In addition to the regular script, there is a separate startup script for operation with GNSS correction:
```bash
docker compose run --rm lio_sam_6axis /workspace/run_lio_sam_gnss.sh
```
*(Or if you changed the `command` in `docker-compose.yml` to `./run_lio_sam_gnss.sh`, a simple `docker compose up` is sufficient).*

This script saves the optimized trajectory (which is exactly aligned with the real GPS points) in the folder `maps_gnss_<Date>/` after completion.

---

## 5. Visualization & Evaluation (Plotly)

To evaluate the results and visually compare different sensor data with each other, the following Python scripts are available:

*   **`extract_origin.py`**: A ROS Python script that can be executed in the container to extract the true initial reference angle after 5 meters of traveled distance (Yaw) and the LLA origin from the raw bag data (`/gps/fix`). The extracted values are saved in `gnss_origin.json` and `raw_gnss.json` (for the plot line).
*   **`plot_comparison.py`**: The main script for evaluation. It runs entirely outside of ROS and only requires standard Python libraries (such as Numpy). It automatically extracts and compares:
    1.  The **Ground Truth** from the `Outdoor1` dataset (automatically filtered and synchronized to LiDAR timestamps).
    2.  The **raw built-in GNSS signal** (from `raw_gnss.json`).
    3.  The free, drifting **LIO-SAM trajectory without GNSS** (`maps/garden_day/optimized_odom_tum.txt`).
    4.  The **GNSS-optimized LIO-SAM trajectory** (from the newest `maps_gnss_...` folder).

The script converts all local LiDAR coordinates back to global GPS coordinates (WGS84 LLA) and automatically calculates correction angles (rotations) based on a 60-meter stretch to perfectly overlay the LiDAR maps and Ground Truth on the real world map. 

The result is finally generated as an interactive HTML map:
**`plot_viewer_map.html`**: This file can easily be opened in any browser (Chrome, Firefox, Safari) by double-clicking. It provides a fully zoomable OpenStreetMap layer on which all lines (Ground Truth, raw GNSS data, LiDAR drift, LiDAR+GNSS optimization) can be interactively toggled on and off.
