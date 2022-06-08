#!/usr/bin/env
import os
import pyrealsense2 as rs
import numpy as np
import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# method to show image
def show_image(img):
    cv.imshow("Image Window", img)
    cv.waitKey(3)


# method to find camera
def find_cam_pos():
    index = 0
    arr = []
    i = 10
    while i > 0:
        cap = cv.VideoCapture(index)
        if cap.read()[0]:
            arr.append(index)
            cap.release()
        index += 1
        i -= 1
    print("Active cameras are", arr)


# main capture loop
def start_camera():
    pub = rospy.Publisher('frames', Image, queue_size=10)
    rospy.init_node('camera_node', anonymous=True)
    rospy.loginfo("Camera node started")

    br = CvBridge()
    # uncomment to find camera positions
    # find_cam_pos()
    video_capture_object = cv.VideoCapture(4)
    video_capture_object.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.25)
    exposure_val = 100
    manual_time = False
    manual_exposure = False
    set_time = 0

    print("test")
    # while the node is running
    while not rospy.is_shutdown():
        ret, frame = video_capture_object.read()

        # if the capture has happened properly
        if ret:
            #testing
            # show_image(frame)
            pub.publish(br.cv2_to_imgmsg(frame))
        else:
            print("capture was false")


if __name__ == '__main__':
    try:
        print("STARTED")
        start_camera()
    except rospy.ROSException:
        print(rospy.ROSException)
