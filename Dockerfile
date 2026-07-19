FROM zhangkin/lio_sam_6axis

COPY scripts/patch.py /patch.py

# Fix the Eigen unaligned array assertion and subsequent segfaults.
# GTSAM was built without AVX, while LIO-SAM was built with -march=native (AVX).
# This causes Eigen ABI mismatch (16-byte vs 32-byte alignment). We remove -march=native.
# We also remove the dense format check which crashes/spams on Ouster data (which inherently has NaNs).
# Install gdb for debugging
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654 && \
    apt-get update && apt-get install -y gdb

RUN cd /root/workspace/src/LIO_SAM_6AXIS/LIO-SAM-6AXIS && \
    sed -i 's/ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");/\/\/ ROS_ERROR.../g' src/imageProjection.cpp && \
    sed -i 's/ros::shutdown();/\/\/ ros::shutdown();/g' src/imageProjection.cpp && \
    sed -i '/float range = pointDistance(thisPoint);/a \            if (!pcl_isfinite(thisPoint.x) || !pcl_isfinite(thisPoint.y) || !pcl_isfinite(thisPoint.z)) continue;' src/imageProjection.cpp && \
    sed -i 's/-march=native //g' CMakeLists.txt && \
    sed -i 's/set(CMAKE_CXX_FLAGS "-std=c++14")/set(CMAKE_CXX_FLAGS "-std=c++14 -DPCL_NO_PRECOMPILE")/g' CMakeLists.txt && \
    sed -i 's/name="$(arg project)_mapOptmization"/name="$(arg project)_mapOptmization" launch-prefix="gdb -batch -ex run -ex bt --args"/g' launch/include/module_loam.launch && \
    python /patch.py

# Build the workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  cd /root/workspace && \
                  catkin clean -y && \
                  catkin build"
