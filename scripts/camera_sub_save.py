#!/usr/bin/env python3

import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def left_callback(image_msg):
    bridge = CvBridge()
    t = image_msg.header.stamp.to_nsec()
    try:
        im = bridge.imgmsg_to_cv2(image_msg)
        cv.imwrite(save_images_path + "/mav0/cam0/data/" + str(t) + ".png", im)

    except CvBridgeError as e:
        print(e)
    pass

def right_callback(image_msg):
    bridge = CvBridge()
    t = image_msg.header.stamp.to_nsec()
    try:
        im = bridge.imgmsg_to_cv2(image_msg)
        cv.imwrite(save_images_path + "/mav0/cam1/data/" + str(t) + ".png", im)

    except CvBridgeError as e:
        print(e)
    pass

def main():
    image_left_sub = rospy.Subscriber("camera/left", Image, left_callback)
    image_right_sub = rospy.Subscriber("camera/right", Image, right_callback)

    rospy.spin()
    

if __name__ == '__main__':
    rospy.init_node("camera_sub_save")
    save_images_path = rospy.get_param("/save_images_path")

    main()