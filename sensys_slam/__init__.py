"""Sensys LiDAR positioning -- downstream georeferencing/evaluation modules.

On the LIO-SAM branch the odometry comes from LIO_SAM_6AXIS; these modules take
the resulting local trajectory and score it against GNSS:

    groundtruth -- load / crop / validity-filter the GNSS reference trajectory
    geo         -- geodetic (lat/lon/alt) <-> local ENU conversions
    align       -- time-sync + SE(3) alignment of the SLAM trajectory to GNSS
    velocity    -- NED-frame velocity from the aligned trajectory
    evaluate    -- RMSE and error-over-time plot vs. ground truth
"""
