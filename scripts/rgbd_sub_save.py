#!/usr/bin/env python3

import cv2 as cv
import rospy
# from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from realsense_hacking.msg import DepthImage

import signal

def signalHandler(singalNumber, frame):
    timestamp_list_str = ""
    for t in timestamps:
        timestamp_list_str += str(t) + " rgb/" + str(t) + ".png " + str(t) + " depth/" + str(t) + ".png\n"
    
    print("Saving association file...")
    timestamps_file = open(save_images_path + "/association.txt", "w")
    timestamps_file.write(timestamp_list_str)
    timestamps_file.close()
    print("Association file saved.")
    

def rgbd_callback(rgbd_image_msg):
    bridge = CvBridge()
    t = rgbd_image_msg.rgb.header.stamp.to_nsec()
    try:
        im_rgb = bridge.imgmsg_to_cv2(rgbd_image_msg.rgb)
        im_depth = bridge.imgmsg_to_cv2(rgbd_image_msg.depth)
        cv.imwrite(save_images_path + "/rgb/" + str(t) + ".png", im_rgb)
        cv.imwrite(save_images_path + "/depth/" + str(t) + ".png", im_depth)

    except CvBridgeError as e:
        print(e)
    else:
        timestamps.append(t)


def main():
    image_rgbd_sub = rospy.Subscriber("camera/rgbd", DepthImage, rgbd_callback)
    rospy.spin()
    

if __name__ == '__main__':
    # handle termination signals>>
    signal.signal(signal.SIGINT, signalHandler)
    signal.signal(signal.SIGTERM, signalHandler)    
    rospy.init_node("camera_sub_save")
    save_images_path = rospy.get_param("/save_images_path")
    timestamps = []
    main()