#!/usr/bin/env
import os

import numpy as np
import rospy
import cv2 as cv
from std_msgs.msg import String
global latest_gps_data
global capture_data
capture_data = np.array([0])
def process_gps_data(data):

    old_gps = ""
    # read latest data
    gps_data = data.data
    # if the gps data has been updated/changed
    if old_gps != gps_data:
        # update latest gps information
        latest_gps_data = gps_data
    old_gps =gps_data
    rospy.loginfo("Latest gps data: %s", latest_gps_data)


def run_camera():
    rospy.init_node("camera_sub_node", anonymous=True)
    rospy.Subscriber('gps_coordinates', String, process_gps_data)



if __name__ == '__main__':
    try:
        run_camera()

        filepath = "~/PycharmProjects/Control-Program-/test"
        # create a video object
        videoCaptureObject = cv.VideoCapture(4)
        videoCaptureObject.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.25)
        exposure_val = 100
        manual_time = False
        manual_exposure = False
        set_time = 0
        print("done xx")
        # while True:

        if capture_data.size == 1:
            capID = 0
        # else add from end
        else:
            capID = capture_data.size + 1

            rospy.loginfo("Capture started:", capID)
            # if a manual time delay has been set get the time and/or exposure values
            # read in a frame
            ret, frame = videoCaptureObject.read()
            # start time to get a positive capture

            # if the capture has happened properly
            if ret:
                # save the frame with data,time as the title
                name = "capture:" + str(capID) + ".png"  # "_lat:" + str(geo.lat) + "_lon:" + str(geo.lon)
                cv.imwrite(os.path.join(filepath, name), frame)
                capID += 1
                print("Finished capture\n")
                # add data to capture data
                capture_data = np.append(capture_data, capID, latest_gps_data)
                # get the time to get  a positive capture
                # wait_time = time_stop - time_start
            # the capture was false
            else:
                print("capture was false")
        rospy.spin()
    except rospy.ROSException:
        pass
