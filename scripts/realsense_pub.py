#!/usr/bin/env python3

from time import time

from os import kill
from sys import path
from typing import Sequence
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import pyrealsense2 as rs

from math import pi


def rotate_cam_pose(pose):
    """Transforms the coordinate system of the camera into a coordinate system with Z up

    Args:
        pose (pyrealsense2.pose): original camera pose

    Returns:
        [pyrealsense2.pose]: transformed camera pose
    """
    #switch translation from (X,Y,Z) to (Z,X,Y)
    pose.translation.x, pose.translation.y, pose.translation.z = pose.translation.z, pose.translation.x, pose.translation.y
    #switch rotation from (X,Y,Z) to (Z,X,Y)
    euler_rot = euler_from_quaternion(
        (pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w))
    q_rot = quaternion_from_euler(euler_rot[2], euler_rot[0], euler_rot[1])
    pose.rotation.x = q_rot[0]
    pose.rotation.y = q_rot[1]
    pose.rotation.z = q_rot[2]
    pose.rotation.w = q_rot[3]

    return pose


def main():
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
    odom_pub = rospy.Publisher("/t265/odom", Odometry, queue_size=10)
    path_pub = rospy.Publisher("/t265/path", Path, queue_size=10)
    # --------------- END_ROS -------------------------

    path = Path()
    #main loop
    while True:
        try:
            #Get frames from realsense
            frames = pipe.wait_for_frames()
            cam_pose = frames.get_pose_frame()

            if cam_pose:
                pose_data = cam_pose.get_pose_data()
                #rotate camera coordinates to point z up
                pose_data = rotate_cam_pose(pose_data)

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
                # --------------- END_ROS -------------------------

        except KeyboardInterrupt:
            print("Shutting down node")
            pipe.stop()
            break


if __name__ == "__main__":
    main()
