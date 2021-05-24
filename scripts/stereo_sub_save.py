#!/usr/bin/env python3

import cv2 as cv
import rospy
# from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from realsense_hacking.msg import StereoImage

import signal

def signalHandler(singalNumber, frame):
    img_list_str = ""
    timestamp_list_str = ""
    for t in timestamps:
        img_list_str += str(t) + "," + str(t) + ".png" + "\n"
        timestamp_list_str += str(t) + "\n"
    
    print("Saving timestamp files...")
    cam0_data_file = open(save_images_path + "/mav0/cam0/data.csv", "w")
    cam0_data_file.write(img_list_str)
    cam0_data_file.close()
    cam1_data_file = open(save_images_path + "/mav0/cam1/data.csv", "w")
    cam1_data_file.write(img_list_str)
    cam1_data_file.close()
    timestamps_file = open(save_images_path + "/mav0/timestamps.txt", "w")
    timestamps_file.write(timestamp_list_str)
    timestamps_file.close()
    print("Timestamp files saved.")
    

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
    else:
        timestamps.append(t)


def main():
    image_stereo_sub = rospy.Subscriber("camera/stereo", StereoImage, stereo_callback)
    rospy.spin()
    

if __name__ == '__main__':
    # handle termination signals>>
    signal.signal(signal.SIGINT, signalHandler)
    signal.signal(signal.SIGTERM, signalHandler)    
    rospy.init_node("camera_sub_save")
    save_images_path = rospy.get_param("/save_images_path")
    timestamps = []
    main()