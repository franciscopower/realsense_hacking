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
save_path_filename = ""

def signalHandler(singalNumber, frame):
    global RUN
    print("Shutting down RealSense Publisher...")
    RUN = False


def saveFile():
    # save_path_filename = "/home/franciscopower/catkin_ws/src/realsense_hacking/CameraTrajectory.txt" 
    while not save_path_filename:
        sleep(0.1)
    f = open(save_path_filename, "a")

    while RUN:
        # print(odom_str + "\n")
        f.write(odom_str + "\n")
        sleep(1) #! Time between consecutive path point recordings

    f.close()


def loadFile(load_path_filename, path):
    if load_path_filename:
        try:
            f = open(load_path_filename, "r")
        except:
            print("No previous path loaded")
            return path
        else:
            #read file
            for l in f.read().split('\n'):
                data = l.split(" ")

                if len(data) == 8:
                    # create odom
                    odom = Odometry()
                    odom.header.stamp = rospy.Time.from_sec(float(data[0]))
                    odom.header.frame_id = "odom"
                    odom.child_frame_id = "base_link"
                    odom.pose.pose.position.x = float(data[1])
                    odom.pose.pose.position.y = float(data[2])
                    odom.pose.pose.position.z = float(data[3])
                    odom.pose.pose.orientation.x = float(data[4])
                    odom.pose.pose.orientation.y = float(data[5])
                    odom.pose.pose.orientation.z = float(data[6])
                    odom.pose.pose.orientation.w = float(data[7])

                    #create ROS pose and path
                    pose = PoseStamped()
                    pose.header = odom.header
                    pose.pose = odom.pose.pose

                    path.header = odom.header
                    path.poses.append(pose)

            print("Loaded previous path")
            return path


def main():
    global odom_str, save_path_filename
    #create realsense pipeline
    pipe = rs.pipeline()
    #configure realsense pipeline
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)

    # Get device ID
    try:
        tm_sensor = cfg.resolve(pipe).get_device().first_pose_sensor()
    except:
        print("RealSense T265 not connected\nShutting Down")
        exit()
    

    # --------------- ROS -------------------------
    #initialize ROS node
    rospy.init_node("t265_data_node")
    #initialize publishers
    odom_broadcaster = tf.TransformBroadcaster()
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    path_pub = rospy.Publisher("path", Path, queue_size=10)

    # get ROS params
    save_path_filename = rospy.get_param('/save_path_file_name') 
    load_path_filename = rospy.get_param('/load_path_file_name') 
    save_map_filename = rospy.get_param('/save_map_file_name') 
    load_map_filename = rospy.get_param('/load_map_file_name') 
    
    path = Path()

    # Load file with previous trajectory
    path = loadFile(load_path_filename, path)
    # --------------- END_ROS -------------------------


    # Load previous Map
    try:
        load_map_file = open(load_map_filename, "r+b")
    except:
        print("No previous map loaded")
    else:
        print("Loaded previous map")
        tm_sensor.import_localization_map(load_map_file.read())
    
    #TODO set notification callback

    #start pipeline
    pipe.start(cfg)

    #Start saveFile thread
    save_th = threading.Thread(None, saveFile)
    save_th.start()

    #main loop
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

                odom_str = str(current_time.to_sec()) \
                    + " " + str(odom.pose.pose.position.x) \
                    + " " + str(odom.pose.pose.position.y) \
                    + " " + str(odom.pose.pose.position.z) \
                    + " " + str(odom.pose.pose.orientation.x) \
                    + " " + str(odom.pose.pose.orientation.y) \
                    + " " + str(odom.pose.pose.orientation.z) \
                    + " " + str(odom.pose.pose.orientation.w)
                # --------------- END_ROS -------------------------
        
    finally:
        save_th.join()

        # save map
        print("Saving Map...")
        bf = open(save_map_filename, "w+b")
        bf.write(bytearray(tm_sensor.export_localization_map()))
        bf.close()
        print("Map Saved")
        
        pipe.stop()


if __name__ == "__main__":
    print("------------ Started RealSense Publisher -------------")

    signal.signal(signal.SIGINT, signalHandler)
    signal.signal(signal.SIGTERM, signalHandler)

    main()
