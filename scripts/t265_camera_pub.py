import pyrealsense2 as rs
import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import signal

from threading import Lock
frame_mutex = Lock()
frame_data = {"left"  : None,
              "right" : None,
              "timestamp_ms" : None
              }

RUN = True

def signalHandler(singalNumber, frame):
    """Handle the SIGINT and SIGTERM signals 
    """
    global RUN
    print("Shutting down RealSense Camera Publisher...")
    #STOP loops
    RUN = False

def callback(frame):
    global frame_data
    if frame.is_frameset():
        frameset = frame.as_frameset()
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        f2 = frameset.get_fisheye_frame(2).as_video_frame()
        left_data = np.asanyarray(f1.get_data())
        right_data = np.asanyarray(f2.get_data())
        ts = frameset.get_timestamp()
        frame_mutex.acquire()
        frame_data["left"] = left_data
        frame_data["right"] = right_data
        frame_data["timestamp_ms"] = ts
        frame_mutex.release()
        

def main():
    rospy.init_node("t265_image_node")
    image_left_pub = rospy.Publisher("camera/left", Image, queue_size=10)
    image_right_pub = rospy.Publisher("camera/right", Image, queue_size=10)
    bridge = CvBridge()
        
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Build config object and stream everything
    cfg = rs.config()

    # Start streaming with our callback
    pipe.start(cfg, callback)

    while RUN:
        # Check if the camera has acquired any frames
        frame_mutex.acquire()
        valid = frame_data["timestamp_ms"] is not None
        frame_mutex.release()

        # If frames are ready to process
        if valid:
            # Hold the mutex only long enough to copy the stereo frames
            frame_mutex.acquire()
            frame_copy = {"left"  : frame_data["left"].copy(),
                            "right" : frame_data["right"].copy()}
            frame_mutex.release()

            try:
                image_left_message = bridge.cv2_to_imgmsg(frame_copy["left"], "passthrough")
                image_right_message = bridge.cv2_to_imgmsg(frame_copy["right"], "passthrough")
            except CvBridgeError as e:
                print(e)
            else:
                image_left_pub.publish(image_left_message)
                image_right_pub.publish(image_right_message)
        
            # cv.imshow("Left image", frame_copy["left"])
            # cv.waitKey(1)

    pipe.stop()

if __name__ == "__main__":
    # handle termination signals
    signal.signal(signal.SIGINT, signalHandler)
    signal.signal(signal.SIGTERM, signalHandler)
    
    main()        