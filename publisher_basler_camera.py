#!/usr/bin/env
import os
import time
import pypylon.pylon as py
import pyrealsense2 as rs
import numpy as np
import rospy
import cv2 as cv
from pypylon import pylon
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

exposure_val = 500


def non_linear_gain(g):
    # Nonlinear function controls the feedback gain. Simpler and faster than the ad-hoc gain adjustment presented in
    # Shim et. al.(2018)
    p = 1.5
    q = 2
    if g < 0.5:
        R = 2
    elif g < 1:
        R = 1 + (2 * (1 - g)) ** p
    elif g < 2:
        R = 1 - 0.5 * (1 - g) ** q
    elif g >= 2:
        R = 0.5
    R = np.log2(R)
    return R


# auto exposure control
def gradient_score(cap, exposure, varargin):
    max_exposure = 10000
    min_exposure = 1
    if len(varargin) > 0:
        max_counter = varargin[0]
    else:
        max_counter = 50

    delta = 0.04
    lambd = 5e2
    Kp = 0.5
    edgewidth = 3

    gamma = np.array([0.50, 0.67, 0.85, 1.00, 1.20, 1.50, 2.00])
    lengamma = len(gamma)

    cap.set(cv.CAP_PROP_EXPOSURE, np.float64(exposure))

    bContinue = True
    LoopCount = 0
    while bContinue:
        LoopCount += 1

        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        else:  # Adjust exposure
            # cv.imshow('frame',frame)
            # Image histogram
            cols = cap.get(cv.CAP_PROP_FRAME_WIDTH)
            rows = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
            ImageY = cv.cvtColor(frame, cv.COLOR_BGR2HSV)[:, :, 2]

            m = np.zeros(lengamma)
            for idx, g in enumerate(gamma):  # Higher g put more weights on stronger edges
                Y = np.power(ImageY, g)
                sobelx = cv.Sobel(Y, cv.CV_64F, 1, 0, ksize=edgewidth)
                sobely = cv.Sobel(Y, cv.CV_64F, 0, 1, ksize=edgewidth)
                GradSq = np.array(sobelx * sobelx + sobely * sobely)
                ImageGrad = GradSq / np.amax(GradSq)
                # cv.imshow('gradient',ImageGrad)
                # cv.waitKey(0)

                bDenoise = ImageGrad > delta
                m[idx] = np.sum(np.log10(lambd * (ImageGrad[bDenoise] - delta) + 1))
                m[idx] /= np.log10(lambd * (1 - delta) + 1)
            # print(m)

            ptbest = np.argmax(m)
            logdE = Kp * non_linear_gain(gamma[ptbest])
            # print(LoopCount,Exposure,Exposure+logdE)
            exposure += logdE
            cap.set(cv.CAP_PROP_EXPOSURE, np.float64(exposure))
            if np.abs(logdE) < 0.2:
                bContinue = False

        if LoopCount > max_counter.all():
            print("Exit Condition: max_counter exceeded")
            break

        elif exposure < min_exposure:
            print("Exit Condition: smaller than min_exposure")
            exposure = min_exposure
            break

        elif exposure > max_exposure:
            print("Exit Condition: max_exposure exceeded")
            exposure = max_exposure
            break

    print('GradientScore iteration count = ', LoopCount)
    print("auto finished:", exposure, "\n")
    return exposure


# method to show image
def show_image(img):
    cv.imshow("Image Window", img)
    cv.waitKey(3)


# method to find camera
def find_num_cameras():
    tlFactory = pylon.TlFactory.GetInstance()
    devices = tlFactory.EnumerateDevices()
    print("Number of Cameras connected: ",len(devices))



# main capture loop
def start_camera():
    global exposure_val
    pub = rospy.Publisher('frames', Image, queue_size=10)
    rospy.init_node('camera_node', anonymous=True)
    rospy.loginfo("Camera node started")

    br = CvBridge()
    # uncomment to find number of cameras connected
    # find_num_cameras()
    # time.sleep(10)
    # connecting to the first available camera
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

    # create a video stream by continually grabbing the latest image
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    converter = pylon.ImageFormatConverter()

    # converting to opencv bgr format
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    # print("test")
    # while the node is running
    while not rospy.is_shutdown():
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        # if the capture has happened properly
        if grabResult.GrabSucceeded():
            # Access the image data and turn into a usable result
            image_converted = converter.Convert(grabResult)
            frame = image_converted.GetArray()
            # testing
            # cv.namedWindow('title', cv.WINDOW_NORMAL)
            # cv.imshow('title', frame)
            # cv.waitKey(1)

            pub.publish(br.cv2_to_imgmsg(frame))
            grabResult.Release()
        else:
            print("capture was false")
    camera.StopGrabbing()


if __name__ == '__main__':
    try:
        print("STARTED")
        start_camera()
    except rospy.ROSException:
        print(rospy.ROSException)
