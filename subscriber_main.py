#!/usr/bin/env
import os
import pyrealsense2 as rs
import numpy as np
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

global latest_gps_data

# process the gps data
def process_gps_data(data):
    old_gps = ""
    # read latest data
    gps_data = data.data
    # if the gps data has been updated/changed
    if old_gps != gps_data:
        # update latest gps information
        latest_gps_data = gps_data
    old_gps = gps_data
    #rospy.loginfo("Latest gps data: %s", latest_gps_data)

#save a frame from a camera
def save_frame(data):
    br = CvBridge()
    frame = br.imgmsg_to_cv2(data)
    cv.imshow("image", frame)
    cv.waitKey(1)

# main node processes
def run_main():
    rospy.init_node("main_node", anonymous=True)
    rospy.loginfo()
    rospy.Subscriber('gps_coordinates', String, process_gps_data)
    rospy.Subscriber('frames', Image, save_frame)


if __name__ == '__main__':
    try:
        run_main()
        rospy.spin()
        # destroy all cv windows on completion
        cv.destroyAllWindows()
    except rospy.ROSException:
        pass
