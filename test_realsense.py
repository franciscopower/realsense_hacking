#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 example             ##
#####################################################

# First import the library
import pyrealsense2 as rs

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.accel)
cfg.enable_stream(rs.stream.gyro)

# Start streaming with requested config
pipe.start(cfg)

try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()
        
        imu = frames.as_motion_frame()
        if imu:
            motion_data = imu.get_motion_data()
            print(motion_data)
        else:
            print("no IMU")
        

        # # Fetch pose frame
        # pose = frames.get_pose_frame()
        # if pose:
        #     # Print some of the pose data to the terminal
        #     data = pose.get_pose_data()
        #     print("Frame #{}".format(pose.frame_number))
        #     print("Position: {}".format(data.translation))
        #     print("Velocity: {}".format(data.velocity))
        #     print("Acceleration: {}\n".format(data.acceleration))

finally:
    pipe.stop()