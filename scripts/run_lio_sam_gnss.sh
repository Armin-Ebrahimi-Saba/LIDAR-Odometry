#!/bin/bash

# Source the LIO-SAM workspace in the docker container
source /opt/ros/melodic/setup.bash 2>/dev/null || true
source /root/workspace/devel/setup.bash 2>/dev/null || true

echo "Injecting use_sim_time into launch file..."
sed -i 's/<launch>/<launch>\n    <param name="\/use_sim_time" value="true" \/>/g' /root/workspace/src/LIO_SAM_6AXIS/LIO-SAM-6AXIS/launch/ouster128_indoors.launch

echo "Patching YAML configuration for Ouster OS0-32 with GNSS enabled..."
python3 /workspace/scripts/patch_yaml.py --gnss


echo "Starting LIO-SAM 6AXIS with Ouster config (GNSS Enabled)..."
# Start LIO-SAM in the background
roslaunch lio_sam_6axis ouster128_indoors.launch &
LIO_PID=$!

echo "Waiting 5 seconds for LIO-SAM to initialize..."
sleep 5

echo "Fixing RViz topics for lio_sam_6axis namespace..."
sed -i 's/\/lio_sam\//\/lio_sam_6axis\//g' /root/workspace/src/LIO_SAM_6AXIS/LIO-SAM-6AXIS/launch/include/config/rviz.rviz

echo "Starting RViz..."
rosrun rviz rviz -d /root/workspace/src/LIO_SAM_6AXIS/LIO-SAM-6AXIS/launch/include/config/rviz.rviz &
RVIZ_PID=$!

echo "Playing rosbag with topic remapping..."
# Play the bag, using --clock to publish /clock if needed, though LIO-SAM uses message timestamps
rosbag play -r 1 bags/lio_sam_ready.bag --clock /ouster/points:=/os_cloud_node/points /ouster/imu_meas:=/stim300/imu/data_raw __name:=bag_player

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAP_DIR="/workspace/output/maps_gnss_${TIMESTAMP}"

echo "Rosbag finished playing. Automatically saving the map to ${MAP_DIR}..."
# Call the ROS service to save the map with NO arguments, it will use savePCDDirectory
rosservice call /lio_sam_6axis/save_map

# Copy the saved maps from the LIO-SAM internal data directory to the host-mounted workspace directory
mkdir -p ${MAP_DIR}
cp -r /root/workspace/src/LIO_SAM_6AXIS/LIO-SAM-6AXIS/data/* ${MAP_DIR}/ || true

echo "Formatting output maps and generating SLAM_path.csv..."
for dir in ${MAP_DIR}/*/; do
    if [ -d "$dir" ]; then
        python3 /workspace/scripts/format_slam_output.py --map_dir "$dir"
    fi
done

echo "Map saved to your ${MAP_DIR}/ folder. Press Ctrl+C to exit."

# Wait for the background ROS launch to finish (when user presses Ctrl+C)
wait $LIO_PID
