#!/usr/bin/env python3

import signal
import numpy as np

import rospy
from realsense_hacking.msg import DepthImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import pyrealsense2 as rs

import cv2 as cv


RUN = True

def signalHandler(singalNumber, frame):
    """Handle the SIGINT and SIGTERM signals 
    """
    global RUN
    print("Shutting down RealSense Publisher...")
    #STOP loops
    RUN = False


def mainLoop():
    """Main loop of program
    """
    global pipe

    #main loop
    try:
        while RUN:
            #Get frames from realsense
            frames = pipe.wait_for_frames()

            f1 = frames.get_color_frame()
            f2 = frames.get_depth_frame()
            if f1 and f2:
                frame_rgb = np.asanyarray(f1.get_data())
                frame_depth = np.asanyarray(f2.get_data())

                bridge = CvBridge()
            
                try:
                    image_message_rgb = bridge.cv2_to_imgmsg(frame_rgb)
                    image_message_depth = bridge.cv2_to_imgmsg(frame_depth)
                except CvBridgeError as e:
                    # print(e)
                    pass
                else:
                    t = frames.get_timestamp()/1000
                    image_message_rgb.header.stamp = rospy.Time.from_sec(t)
                    image_message_depth.header.stamp = rospy.Time.from_sec(t)

                    depth_image_message = DepthImage()
                    depth_image_message.rgb = image_message_rgb
                    depth_image_message.depth = image_message_depth
                    image_rgbd_pub.publish(depth_image_message)

                    image_rgb_pub.publish(image_message_rgb)
                    image_depth_pub.publish(image_message_depth)

                    # cv.imshow("depth", frame_depth)
                    # cv.imshow("color", frame_rgb)
                    # cv.waitKey(1)

    finally:
        pipe.stop()


if __name__ == "__main__":
    print("\n------------ Started RealSense Publisher -------------")

    # handle termination signals
    signal.signal(signal.SIGINT, signalHandler)
    signal.signal(signal.SIGTERM, signalHandler)

    #create realsense pipeline
    pipe = rs.pipeline()
    #configure realsense pipeline
    cfg = rs.config()
    # cfg.enable_stream(rs.stream.pose)

    # --------------- ROS -------------------------
    #initialize ROS node
    rospy.init_node("d415_data_node")
    #initialize publishers
    image_rgbd_pub = rospy.Publisher("camera/rgbd", DepthImage, queue_size=1)
    image_rgb_pub = rospy.Publisher("camera/rgb", Image, queue_size=1)
    image_depth_pub = rospy.Publisher("camera/depth", Image, queue_size=1)

    # --------------- END_ROS -------------------------
    
    #start pipeline
    pipe.start(cfg)

    mainLoop()
