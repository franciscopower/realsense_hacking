#!/usr/bin/env python3

import cv2 as cv
import rospy
# from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from realsense_hacking.msg import StereoImage

def stereo_callback(stereo_image_msg):
    bridge = CvBridge()
    t = stereo_image_msg.left.header.stamp.to_nsec()
    try:
        im_left = bridge.imgmsg_to_cv2(stereo_image_msg.left)
        im_right = bridge.imgmsg_to_cv2(stereo_image_msg.right)
        cv.imwrite(save_images_path + "/mav0/cam0/data/" + str(t) + ".png", im_left)
        cv.imwrite(save_images_path + "/mav0/cam1/data/" + str(t) + ".png", im_right)

    except CvBridgeError as e:
        print(e)
    pass


def main():
    image_stereo_sub = rospy.Subscriber("camera/stereo", StereoImage, stereo_callback)
    rospy.spin()
    

if __name__ == '__main__':
    rospy.init_node("camera_sub_save")
    save_images_path = rospy.get_param("/save_images_path")
    main()