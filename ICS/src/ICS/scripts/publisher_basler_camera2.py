#!/usr/bin/env
import time

import numpy as np
import rospy
import cv2 as cv
from pypylon import pylon
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

max_exposure = 1000000
min_exposure = 16
exposure_val = 8000
converter = pylon.ImageFormatConverter()
# set converter to opencv bgr format
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned


def get_min_max(camera):
    global max_exposure
    global min_exposure
    max_exposure = camera.ExposureTime.Max
    min_exposure = camera.ExposureTime.Min
    cols = camera.Width.GetValue()
    rows = camera.Height.GetValue()
    print("max: ", max_exposure)
    print("min: ", min_exposure)
    print("Cols: ", cols)
    print("rows: ", rows)
    time.sleep(5)


# method to show image
def show_image(img):
    cv.namedWindow('Image Window 2', cv.WINDOW_NORMAL)
    cv.imshow("Image Window 2", img)
    cv.waitKey(3)


# method to find camera
def find_num_cameras():
    tlFactory = pylon.TlFactory.GetInstance()
    devices = tlFactory.EnumerateDevices()
    print("Number of Cameras connected: ", len(devices))


# main capture loop
def start_camera(data):
    global exposure_val
    pub = rospy.Publisher('frames2', Image, queue_size=10)

    br = CvBridge()
    # uncomment to find number of cameras connected
    # find_num_cameras()
    # time.sleep(10)

    # connecting to the second camera
    tl_factory = pylon.TlFactory.GetInstance()
    devices = tl_factory.EnumerateDevices()
    camera = pylon.InstantCamera()
    camera.Attach(tl_factory.CreateDevice(devices[1]))
    camera.Open()
    # create a video stream by continually grabbing the latest image
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    # create a image converter
    converter = pylon.ImageFormatConverter()
    camera.AutoFunctionROIUseBrightness.SetValue(True)
    camera.ExposureAuto.SetValue("Continuous")
    # set converter to opencv bgr format
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
    # get max/min exposuretimes
    # get_min_max(camera)

    # while the node is running
    while not rospy.is_shutdown():
        grabResult = camera.RetrieveResult(500, pylon.TimeoutHandling_ThrowException)
        # if the capture has happened properly
        if grabResult.GrabSucceeded():
            # Access the image data and turn into a usable result
            image_converted = converter.Convert(grabResult)
            frame = image_converted.GetArray()
            # publish the frame
            pub.publish(br.cv2_to_imgmsg(frame))
            # uncomment to see camera feed
            show_image(frame)
            grabResult.Release()
        else:
            print("capture was false")
    camera.StopGrabbing()
    camera.Close()


def shut_down(data):
    if not data.data:
        rospy.signal_shutdown("shutdown called ")


def main():
    rospy.init_node('camera_node', anonymous=True)
    rospy.loginfo("Camera node 2 started")
    rospy.Subscriber('shutdown', Bool, shut_down)
    rospy.Subscriber('start', Bool, start_camera)


if __name__ == '__main__':
    try:
        time.sleep(1)
        main()
        rospy.spin()
    except rospy.ROSException:
        print(rospy.ROSException)
