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
static_node_pose = None

def signalHandler(singalNumber, frame):
    global RUN
    print("Shutting down RealSense Publisher...")
    RUN = False
    #TODO place map saving function and thread join here


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


def loadPrevPath(load_path_filename, prev_path):
    
    if load_path_filename:
        try:
            f = open(load_path_filename, "r")
        except:
            print("No previous path loaded")
            return prev_path
        else:
            #read file
            for l in f.read().split('\n'):
                data = l.split(" ")

                if len(data) == 8:
                    # create odom
                    prev_odom = Odometry()
                    prev_odom.header.stamp = rospy.Time.from_sec(float(data[0]))
                    prev_odom.header.frame_id = "prev_origin"
                    prev_odom.child_frame_id = "camera"
                    prev_odom.pose.pose.position.x = float(data[1])
                    prev_odom.pose.pose.position.y = float(data[2])
                    prev_odom.pose.pose.position.z = float(data[3])
                    prev_odom.pose.pose.orientation.x = float(data[4])
                    prev_odom.pose.pose.orientation.y = float(data[5])
                    prev_odom.pose.pose.orientation.z = float(data[6])
                    prev_odom.pose.pose.orientation.w = float(data[7])

                    #create ROS pose and path
                    prev_pose = PoseStamped()
                    prev_pose.header = prev_odom.header
                    prev_pose.pose = prev_odom.pose.pose

                    prev_path.header = prev_odom.header
                    prev_path.poses.append(prev_pose)

            print("Loaded previous path")
    
    return prev_path


def notificationCallback(notification):
    global map_transform
    
    if notification.get_category() == rs.notification_category.pose_relocalization:
        print("Relocalization Event Detected")

        print(tm_sensor.get_static_node("static_node")[0])
        print(tm_sensor.get_static_node("static_node")[1])
        print(tm_sensor.get_static_node("static_node")[2])

        if tm_sensor.get_static_node("static_node")[0]:
            map_transform = [
                (
                    -tm_sensor.get_static_node("static_node")[1].z,
                    -tm_sensor.get_static_node("static_node")[1].x,
                    tm_sensor.get_static_node("static_node")[1].y
                ),
                (
                    -tm_sensor.get_static_node("static_node")[2].z,
                    -tm_sensor.get_static_node("static_node")[2].x,
                    tm_sensor.get_static_node("static_node")[2].y,
                    tm_sensor.get_static_node("static_node")[2].w
                )
            ]
            print("Applied map transformation")
    

def mainLoop():
    global odom_str, path, pipe, tm_sensor

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
                #broadcast transform origin --> camera
                odom_broadcaster.sendTransform(
                    (pose_data.translation.x, pose_data.translation.y,
                     pose_data.translation.z),
                    (pose_data.rotation.x, pose_data.rotation.y,
                     pose_data.rotation.z, pose_data.rotation.w),
                    current_time,
                    "camera",
                    "origin"
                )

                #broadcast transform prev_origin --> origin
                odom_broadcaster.sendTransform(
                    map_transform[0],
                    map_transform[1],
                    current_time,
                    "prev_origin",
                    "origin",
                )

                #create ROS odometry
                odom = Odometry()
                odom.header.stamp = current_time
                odom.header.frame_id = "origin"
                odom.child_frame_id = "camera"
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
                prev_path_pub.publish(prev_path)
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
    prev_path_pub = rospy.Publisher("prev_path", Path, queue_size=10)

    # get ROS params
    save_path_filename = rospy.get_param('/save_path_file_name') 
    load_path_filename = rospy.get_param('/load_path_file_name') 
    save_map_filename = rospy.get_param('/save_map_file_name') 
    load_map_filename = rospy.get_param('/load_map_file_name') 
    
    path = Path()
    prev_path = Path()

    # Load file with previous trajectory
    prev_path = loadPrevPath(load_path_filename, prev_path)
    # --------------- END_ROS -------------------------


    # Load previous Map
    try:
        load_map_file = open(load_map_filename, "r+b")
    except:
        print("No previous map loaded")
        static_node_bool = False
    else:
        print("Loaded previous map")
        tm_sensor.import_localization_map(list(load_map_file.read()))
        static_node_bool = True
    
    #set notification callback
    tm_sensor.set_notifications_callback(notificationCallback)

    #start pipeline
    pipe.start(cfg)

    #Start saveFile thread
    save_th = threading.Thread(None, saveFile)
    save_th.start()

    #default transform between previous map and new map
    map_transform = [(0,0,0), (0,0,0,1)] 

    origin_pos = rs.vector()
    origin_pos.x = 0
    origin_pos.y = 0
    origin_pos.z = 0
    origin_q = rs.quaternion()
    origin_q.x = 0
    origin_q.y = 0
    origin_q.z = 0
    origin_q.w = 1
    
    print("Attempting to adquire static node... ")
    while (not static_node_bool) : #TODO: add "and RUN"
        static_node_bool = tm_sensor.set_static_node("static_node", origin_pos, origin_q)
    print("Static Node: " + str(static_node_bool))

    mainLoop()
