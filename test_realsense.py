#!/usr/bin/env python3
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 rpy example         ##
#####################################################

# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2 as cv

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()

tm_sensor = cfg.resolve(pipe).get_device()
print(tm_sensor)

# Start streaming with requested config
pipe.start(cfg)

colorizer = rs.colorizer()

try:
    while (True):
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        
        if color:
            color_image = np.asanyarray(color.get_data())
            cv.imshow("color", color_image)

        if depth:
            depth_color = colorizer.colorize(depth)
            depth_color_color = np.asanyarray(depth_color.get_data())
            cv.imshow("depth", depth_color_color)
            cv.imshow("other depth", np.asanyarray(depth.get_data()))

        k = cv.waitKey(1)
        if k == ord('q'):
            break


finally:
    pipe.stop()