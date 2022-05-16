import time
import os
import pyrealsense2 as rs
import numpy as np
import sys
import cv2 as cv
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from CaptureWindow import Ui_MainWindow
import serial
from ublox_gps import UbloxGps

# global array of capture data
capture_data = np.array(int)


def NonlinearGain(self, g):
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


def GradientScore(self, cap, Exposure, varargin):
    MAX_EXPOSURE = 200000
    MIN_EXPOSURE = 1.0
    if len(varargin) > 0:
        MAX_COUNTER = varargin[0]
    else:
        MAX_COUNTER = 50

    delta = 0.04
    lambd = 5e2
    Kp = 0.5
    edgewidth = 3

    gamma = np.array([0.50, 0.67, 0.85, 1.00, 1.20, 1.50, 2.00])
    lengamma = len(gamma)

    cap.set(cv.CAP_PROP_EXPOSURE, np.float64(Exposure))

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
            logdE = Kp * NonlinearGain(self, gamma[ptbest])
            # print(LoopCount,Exposure,Exposure+logdE)
            Exposure += logdE
            cap.set(cv.CAP_PROP_EXPOSURE, np.float64(Exposure))
            if np.abs(logdE) < 0.2:
                bContinue = False

        if LoopCount > MAX_COUNTER.all():
            print("maximum number of iteration has been exceeded.")
            break
        elif Exposure < MIN_EXPOSURE:
            print("min exposure exceeded")
            break
        elif Exposure > MAX_EXPOSURE:
            print("max exposure exceeded")
            break

    print('GradientScore iteration count = ', LoopCount)
    return Exposure


class capture_window(QMainWindow, Ui_MainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

    # method that opens a file dialog for folder selection
    def getFolder(self):
        dialog = QFileDialog(self)
        filename = dialog.getExistingDirectory(
            self, "Select a folder to save capture to")
        return filename

    # method to read in all cameras and prints out the max and min values of exposure
    def setUp(self):
        # checks the first 10 indexes.
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
        cfg = rs.config()
        p = rs.pipeline()
        profile = p.start(cfg)
        sensor = profile.get_device().query_sensors()[0]
        max_exp = sensor.get_option_range(rs.option.exposure).max
        min_exp = sensor.get_option_range(rs.option.exposure).min
        print("max exposure:", max_exp)
        print("min exposure:", min_exp)

    # method to manual change the parameters
    def adjustParams(self):
        # get exposure value from textbox
        exposure_val = self.textAdjust.text()
        return exposure_val

    # method to manually set the timing
    def adjustTiming(self):
        time_set = self.timingAdjust.text()
        return time_set

    # method for the capturing of images from the camera
    def startCapture(self):

        if capwindow.startButton.isChecked():
            # get the file path
            filepath = capture_window.getFolder(self)
            print(filepath)
            # create a thread
            self.thread = QThread()
            # create the capture loop class as a worker passing the file path and numpy array
            self.worker = capture_loop(filepath)
            # move worker to a thread
            self.worker.moveToThread(self.thread)
            # connect signals and slots
            # when started thread runs runCapture
            self.thread.started.connect(self.worker.runCapture)
            self.worker.finished.connect(self.thread.quit)
            # make sure that the worker and thread delete themselves on finished signal
            self.worker.finished.connect(self.worker.deleteLater)
            self.thread.finished.connect(self.thread.deleteLater)
            self.thread.start()


class capture_loop(QObject):
    def __init__(self, path):
        super().__init__()
        self.filename = path

    finished = pyqtSignal()

    # task for threading (capture photos loop)
    def runCapture(self):
        # initalize gps
        port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=20)
        gps = UbloxGps(port)
        # if capture data is empty
        global capture_data
        if capture_data.size == 1:
            capID = 0
        # else add from end
        else:
            capID = capture_data.size + 1

        # get the save location (path) using a file dialog
        filepath = self.filename
        # create a video object
        videoCaptureObject = cv.VideoCapture(4)
        videoCaptureObject.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.25)
        exposure_val = 1
        manual_time = False
        manual_exposure = False
        set_time = 0
        # while the startCapture button is checked (capture loop)
        while capwindow.startButton.isChecked():

            print("capture started:", capID)
            # if a manual time delay has been set get the time and/or exposure values
            if capwindow.timingButton.isChecked():
                set_time = capwindow.adjustTiming()
                set_time = int(set_time)
                print(set_time)
                manual_time = True
            else:
                manual_time = False

            if capwindow.adjustCamera.isChecked():
                # set defined exposure value
                exposure_val = capwindow.adjustParams()
                videoCaptureObject.set(cv.CAP_PROP_EXPOSURE, float(exposure_val))
                manual_exposure = True
            else:
                manual_exposure = False
            # read in a frame
            ret, frame = videoCaptureObject.read()
            # start time to get a positive capture
            time_start = time.time()
            # if the capture has happened properly
            if ret:
                # get the gps coordinates of the capture
                geo = gps.geo_coords()
                # print them
                print("Longitude: ", geo.lon)
                print("Latitude: ", geo.lat)
                # stop timer as a positive capture has happened
                time_stop = time.time()
                # if the value has not been overwritten
                if not manual_exposure:
                    test = videoCaptureObject.get(cv.CAP_PROP_EXPOSURE)
                    # test2 = videoCaptureObject.get(cv.CAP_PROP_AUTO_EXPOSURE)
                    print("Exposure value auto:", test, "\n")
                    # run gradient score
                    exposure_val = GradientScore(capwindow, videoCaptureObject, exposure_val, frame)
                    # print("Auto Exposure value:", exposure_val)

                else:
                    test = videoCaptureObject.get(cv.CAP_PROP_EXPOSURE)
                    print("Manual Exposure value:", test)
                # save the frame with data,time as the title
                name = "capture:" + str(capID) + "_lat:" + str(geo.lat) + "_lon:" + str(geo.lon) + ".png"
                cv.imwrite(os.path.join(filepath, name), frame)
                capID += 1
                # add data to capture data (add gps data here too)
                capture_data = np.append(capture_data, [capID, geo.lat, geo.lon])
                # get the time to get  a positive capture
                wait_time = time_stop - time_start
                if manual_time:
                    time.sleep(set_time)
                    print("waiting for:", set_time, " seconds")
                else:
                    # wait for auto time
                    time.sleep(wait_time)
            # the capture was false
            else:
                print("capture was false")
        # close gps device
        port.close()
        self.finished.emit()


# main code of the application#
if __name__ == "__main__":
    app = QApplication(sys.argv)
    capwindow = capture_window()
    capwindow.show()
    sys.exit(app.exec_())
