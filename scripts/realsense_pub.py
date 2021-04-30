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


def main():
    pipe = rs.pipeline()

    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)

    pipe.start(cfg)

    rospy.init_node("t265_data_node")
    odom_broadcaster = tf.TransformBroadcaster()
    odom_pub = rospy.Publisher("/t265/odom", Odometry, queue_size=10)
    path_pub = rospy.Publisher("/t265/path", Path, queue_size=10)

    path = Path()
    while True:
        try:
            current_time = rospy.Time.now()

            frames = pipe.wait_for_frames()
            cam_pose = frames.get_pose_frame()
            if cam_pose:
                pose_data = cam_pose.get_pose_data()

                pose_translation = (pose_data.translation.z,
                                    pose_data.translation.x,
                                    pose_data.translation.y)

                pose_rotation = (pose_data.rotation.x,
                                 pose_data.rotation.y,
                                 pose_data.rotation.z,
                                 pose_data.rotation.w)
                pose_e = euler_from_quaternion(pose_rotation)
                pose_rotation = quaternion_from_euler(pose_e[2], pose_e[0], pose_e[1])

                odom_broadcaster.sendTransform(
                    pose_translation,
                    pose_rotation,
                    current_time,
                    "base_link",
                    "odom"
                )

                odom = Odometry()
                odom.header.stamp = current_time
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = pose_translation[0]
                odom.pose.pose.position.y = pose_translation[1]
                odom.pose.pose.position.z = pose_translation[2]
                odom.pose.pose.orientation.x = pose_rotation[0]
                odom.pose.pose.orientation.y = pose_rotation[1]
                odom.pose.pose.orientation.z = pose_rotation[2]
                odom.pose.pose.orientation.w = pose_rotation[3]
                odom.twist.twist.linear = pose_data.velocity
                odom.twist.twist.angular = pose_data.angular_velocity

                pose = PoseStamped()
                pose.header = odom.header
                pose.pose = odom.pose.pose

                path.header = odom.header
                path.poses.append(pose)

                path_pub.publish(path)
                odom_pub.publish(odom)

        except KeyboardInterrupt:
            print("Shutting down node")
            pipe.stop()
            break


if __name__ == "__main__":
    main()
