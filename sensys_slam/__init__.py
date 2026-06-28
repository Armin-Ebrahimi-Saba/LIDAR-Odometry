"""Sensys LiDAR positioning pipeline.

Modules:
    timestamps    -- pair .laz scan files with bag-recorded timestamps
    lidar_io      -- load .laz / PointCloud2 scans for the odometry loop
    odometry      -- drive the KISS-ICP package over a scan sequence -> poses + 3D map
    groundtruth   -- load / crop / validity-filter the GNSS reference trajectory
    geo           -- geodetic (lat/lon/alt) <-> local ENU conversions
    frames        -- body-frame (Pixhawk FRD) definition + LiDAR->body extrinsic
    align         -- time-sync + start-anchored SE(3) georeferencing to GNSS
    velocity      -- NED-frame velocity from the georeferenced trajectory
    evaluate      -- RMSE and error-over-time plot vs. ground truth
    attitude      -- optional PX4-attitude scan deskewer (lidar.imu_deskew)
"""
