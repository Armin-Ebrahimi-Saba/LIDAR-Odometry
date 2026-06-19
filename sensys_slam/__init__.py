"""Sensys LiDAR positioning pipeline.

Modules:
    timestamps  -- pair .laz scan files with bag-recorded timestamps
    lidar_io    -- load .laz point clouds, dataset wrapper for the odometry loop
    odometry    -- run KISS-ICP odometry/SLAM over a sequence of scans
    groundtruth -- load / crop / validity-filter the GNSS reference trajectory
    geo         -- geodetic (lat/lon/alt) <-> local ENU conversions
    align       -- time-sync + SE(3) alignment of the SLAM trajectory to GNSS
    velocity    -- NED-frame velocity from the aligned trajectory
    evaluate    -- RMSE and error-over-time plot vs. ground truth
    imu_assist  -- optional: IMU-attitude-seeded initial guess for ICP
"""
