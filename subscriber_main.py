#!/usr/bin/env
import os
import pyrealsense2 as rs
import numpy as np
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

filepath1 = "/home/aaeon/PycharmProjects/Control-Program-/test"
filepath2 = "/home/aaeon/PycharmProjects/Control-Program-/test2"
latest_gps_data = ''
cap_id = 0


# process the gps data
def process_gps_data(data):
    old_gps = ""
    # read latest data
    gps_data = data.data
    # if the gps data has been updated/changed
    if old_gps != gps_data:
        # update latest gps information
        global latest_gps_data
        latest_gps_data = gps_data
    old_gps = gps_data
    # rospy.loginfo("Latest gps data: %s", latest_gps_data)


# save a frame from a camera
def save_frame1(data):
    br = CvBridge()
    frame = br.imgmsg_to_cv2(data)
    # testing
    # cv.imshow("image", frame)
    # cv.waitKey(1)
    global cap_id
    name = "capture_" + str(cap_id) + " " + latest_gps_data + ".png"
    cv.imwrite(os.path.join(filepath1, name), frame)
    print(os.path.join(filepath1, name))
    # print("Capture " + str(cap_id) + " saved")
    cap_id += 1


def save_frame2(data):
    br = CvBridge()
    frame = br.imgmsg_to_cv2(data)
    # testing
    # cv.imshow("image", frame)
    # cv.waitKey(1)
    global cap_id
    name = "capture_" + str(cap_id) + " " + latest_gps_data + ".png"
    cv.imwrite(os.path.join(filepath2, name), frame)
    print(os.path.join(filepath2, name))
    # print("Capture " + str(cap_id) + " saved")
    cap_id += 1


# main node processes
def run_main():
    rospy.init_node("main_node", anonymous=True)

    rospy.loginfo("main node started")

    rospy.Subscriber('gps_coordinates', String, process_gps_data)
    rospy.Subscriber('frames', Image, save_frame1)
    rospy.Subscriber('frames2', Image, save_frame2)


if __name__ == '__main__':
    try:
        run_main()
        rospy.spin()
        # destroy all cv windows on completion
        cv.destroyAllWindows()
    except rospy.ROSException:
        pass
