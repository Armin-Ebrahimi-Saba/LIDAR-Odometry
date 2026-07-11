#!/bin/bash

# Source the LIO-SAM workspace in the docker container
source /opt/ros/melodic/setup.bash 2>/dev/null || true
source /root/workspace/devel/setup.bash 2>/dev/null || true

echo "Injecting use_sim_time into launch file..."
sed -i 's/<launch>/<launch>\n    <param name="\/use_sim_time" value="true" \/>/g' /root/workspace/src/LIO_SAM_6AXIS/LIO-SAM-6AXIS/launch/ouster128_indoors.launch

echo "Patching YAML configuration for Ouster OS0-32..."
python3 /workspace/patch_yaml.py


echo "Starting LIO-SAM 6AXIS with Ouster config..."
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
rosbag play lio_sam_ready.bag --clock /ouster/points:=/os_cloud_node/points /ouster/imu_meas:=/stim300/imu/data_raw

echo "Rosbag finished playing. You can now save the map in another terminal using:"
echo "rosservice call /lio_sam_6axis/save_map"
echo "Press Ctrl+C to exit."

# Wait for the background ROS launch to finish (when user presses Ctrl+C)
wait $LIO_PID
