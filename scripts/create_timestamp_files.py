#!/usr/bin/env python3

import os

session_name = "session"

cam0_path = "/home/franciscopower/catkin_ws/src/realsense_hacking/output/" + session_name + "/mav0/cam0/data"
cam1_path = "/home/franciscopower/catkin_ws/src/realsense_hacking/output/" + session_name + "/mav0/cam1/data"

valid_images = []


for file in os.listdir(cam0_path):
    if file in os.listdir(cam1_path):
       valid_images.append(file)

# print(valid_images)
print("Total images in cam0: " + str(len(os.listdir(cam0_path))))
print("Total images in cam1: " + str(len(os.listdir(cam1_path))))
print("Valid images: " + str(len(valid_images)))

str = "\#timestamp [ns],filename\n"
for i in valid_images:
    str += i.split(".")[0] + "," + i + "\n"

cam0_data_file = open("/home/franciscopower/catkin_ws/src/realsense_hacking/output/" + session_name + "/mav0/cam0/data.csv", "w")
cam0_data_file.write(str)
cam0_data_file.close()
cam1_data_file = open("/home/franciscopower/catkin_ws/src/realsense_hacking/output/" + session_name + "/mav0/cam1/data.csv", "w")
cam1_data_file.write(str)
cam1_data_file.close()

str_timestamp = ""
for i in valid_images:
    str_timestamp += i.split(".")[0] + "\n"

timestamps_file = open("/home/franciscopower/catkin_ws/src/realsense_hacking/output/" + session_name + "/mav0/timestamps.txt", "w")
timestamps_file.write(str_timestamp)
timestamps_file.close()



