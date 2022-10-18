#!/usr/bin/env
import csv
import datetime
import os
import re
import rospy
import cv2 as cv
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

filepath1 = "/home/aaeon/Desktop/test1"
filepath2 = "/home/aaeon/Desktop/test2"
latest_gps_data = ''
old_gps = ""
cap_id = 0
data_csv = []
headings = ["capture id", "lat", "lon", "filepath", "capture_date", "capture_time"]


# process the gps data
def process_gps_data(data):
    global old_gps
    global latest_gps_data
    # read latest data
    gps_data = data.data
    gps_data = str(gps_data)
    # if the gps data has been updated/changed
    if old_gps != gps_data:
        # update latest gps information
        latest_gps_data = gps_data
        old_gps = gps_data
        # split gps into lat and lon
        coord = latest_gps_data.split("#")
        rospy.loginfo("Latest gps data: %s", coord[0])


# save a frame from a camera
def save_frame1(data):
    br = CvBridge()
    frame = br.imgmsg_to_cv2(data)
    # testing
    # cv.imshow("image", frame)
    # cv.waitKey(1)
    # get data and time
    current_date = datetime.date.today()
    t = datetime.datetime.now()
    time_current = str(t.hour) + ':' + str(t.minute) + ':' + str(t.second)
    global latest_gps_data
    global cap_id
    global data_csv
    coordinates = str(latest_gps_data)
    list = coordinates.split("#")
    rospy.loginfo("coordinates: %s", coordinates)
    name = "capture_" + str(cap_id) + "_lat:" + list[0] + "_lon: " + list[1] + ".jpeg"
    save_location = os.path.join(filepath1, name)
    cv.imwrite(save_location, frame)
    print(os.path.join(filepath1, name))
    # print("Capture " + str(cap_id) + " saved")
    data_csv.append([cap_id, list[0], list[1], save_location, current_date, time_current])
    cap_id += 1


def save_frame2(data):
    br = CvBridge()
    frame = br.imgmsg_to_cv2(data)
    # get data and time
    current_date = datetime.date.today()
    t = datetime.datetime.now()
    time_current = str(t.hour) + ':' + str(t.minute) + ':' + str(t.second)
    # testing
    # cv.imshow("image", frame)
    # cv.waitKey(1)
    global latest_gps_data
    global cap_id
    global data_csv
    coordinates = str(latest_gps_data)
    list_coord = coordinates.split("#")
    rospy.loginfo("coordinates: %s", coordinates)
    name = "capture_" + str(cap_id) + "_lat:" + list_coord[0] + "_lon:" + list_coord[1] + ".jpeg"
    save_location = os.path.join(filepath2, name)
    cv.imwrite(save_location, frame)
    print(os.path.join(filepath2, name))
    data_csv.append([cap_id, list_coord[0], list_coord[1], save_location, current_date, time_current])
    # print("Capture " + str(cap_id) + " saved")
    cap_id += 1


def set_filepath(data):
    path = str(data)
    path = path.replace('\"', '')
    fp = re.split("#| ", path)
    print(fp)
    global filepath1
    global filepath2
    filepath1 = fp[1]
    filepath2 = fp[2]

    print("filepath 1: ", filepath1)
    print("filepath 2: ", filepath2)


def shut_down(data):
    if not data.data:
        rospy.loginfo("shutting down main node")
        global filepath1
        filepath = filepath1 + "/all_data.csv"
        with open(filepath, 'w') as f:
            # using csv.writer method from CSV package
            write = csv.writer(f)
            write.writerow(headings)
            write.writerows(data_csv)

        rospy.signal_shutdown("shutdown called ")


def run_main():
    rospy.init_node("main_node", anonymous=True)
    rospy.loginfo("main node started")
    rospy.Subscriber('filepaths', String, set_filepath)
    rospy.Subscriber('gps_coordinates', String, process_gps_data)
    rospy.Subscriber('frames', Image, save_frame1)
    rospy.Subscriber('frames2', Image, save_frame2)
    rospy.Subscriber('shutdown', Bool, shut_down)


if __name__ == '__main__':
    try:
        run_main()
        rospy.spin()
        # destroy all cv windows on completion
        cv.destroyAllWindows()
    except rospy.ROSException:
        pass
