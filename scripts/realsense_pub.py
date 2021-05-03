#!/usr/bin/env python3

import signal
import threading
from time import sleep

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf

import pyrealsense2 as rs


RUN = True
odom_str = ""
filename = ""

def signalHandler(singalNumber, frame):
    global RUN
    print(" handling signal ")
    RUN = False

def saveFile():
    # filename = "/home/franciscopower/catkin_ws/src/realsense_hacking/CameraTrajectory.txt" 
    while not filename:
        sleep(0.1)
        
    f = open(filename, "a")

    while RUN:
        # print(odom_str + "\n")
        f.write(odom_str + "\n")
        sleep(1)

    f.close()
        

def main():
    global odom_str, filename
    #create realsense pipeline
    pipe = rs.pipeline()
    #configure realsense pipeline
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)
    #start pipeline
    pipe.start(cfg)

    # --------------- ROS -------------------------
    #initialize ROS node
    rospy.init_node("t265_data_node")
    #initialize publishers
    odom_broadcaster = tf.TransformBroadcaster()
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    path_pub = rospy.Publisher("path", Path, queue_size=10)

    filename = rospy.get_param('/save_file_name_param') 
    # --------------- END_ROS -------------------------

    path = Path()
    #main loop
    odom_str = ""
    try:
        while RUN:
            #Get frames from realsense
            frames = pipe.wait_for_frames()
            cam_pose = frames.get_pose_frame()

            if cam_pose:
                pose_data = cam_pose.get_pose_data()
                #rotate camera coordinates to point z up
                pose_data.translation.x, pose_data.translation.y, pose_data.translation.z = -pose_data.translation.z, -pose_data.translation.x, pose_data.translation.y
                pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z, pose_data.rotation.w = -pose_data.rotation.z, -pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.w

                # --------------- ROS -------------------------
                current_time = rospy.Time.now()
                #broadcast transform odom --> baselink
                odom_broadcaster.sendTransform(
                    (pose_data.translation.x, pose_data.translation.y,
                     pose_data.translation.z),
                    (pose_data.rotation.x, pose_data.rotation.y,
                     pose_data.rotation.z, pose_data.rotation.w),
                    current_time,
                    "base_link",
                    "odom"
                )

                #create ROS odometry
                odom = Odometry()
                odom.header.stamp = current_time
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.position = pose_data.translation
                odom.pose.pose.orientation = pose_data.rotation
                odom.twist.twist.linear = pose_data.velocity
                odom.twist.twist.angular = pose_data.angular_velocity

                #create ROS pose and path
                pose = PoseStamped()
                pose.header = odom.header
                pose.pose = odom.pose.pose

                path.header = odom.header
                path.poses.append(pose)

                #publish ROS path and odom
                path_pub.publish(path)
                odom_pub.publish(odom)

                odom_str = str(odom.header.stamp) \
                    + " " + str(odom.pose.pose.position.x) \
                    + " " + str(odom.pose.pose.position.y) \
                    + " " + str(odom.pose.pose.position.z) \
                    + " " + str(odom.pose.pose.orientation.x) \
                    + " " + str(odom.pose.pose.orientation.y) \
                    + " " + str(odom.pose.pose.orientation.z) \
                    + " " + str(odom.pose.pose.orientation.w)
                # --------------- END_ROS -------------------------
        
    finally:
        print("Shutting down node")
        pipe.stop()


if __name__ == "__main__":
    print("------------ Started RealSense Publisher -------------")

    signal.signal(signal.SIGINT, signalHandler)
    signal.signal(signal.SIGTERM, signalHandler)

    save_th = threading.Thread(None, saveFile)
    save_th.start()

    main()

    save_th.join()