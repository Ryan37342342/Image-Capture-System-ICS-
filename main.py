import datetime
import time
import os
import numpy as np
import sys
import cv2 as cv
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QFileDialog
from CaptureWindow import Ui_MainWindow


# class of the capture GUI#
class capture_window(QMainWindow, Ui_MainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

    def getFolder(self):
        dialog = QFileDialog(self)
        filename = dialog.getExistingDirectory(self, "Select a file")
        return filename


    # method to read in all cameras
    def setupCamera(self):
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
    #method to manual change the parameters 
    def adjustParams(self):
        #get exposure value from textbox
        exposure_val= self.textAdjust.text()
        print("exposure value manually set to:",exposure_val)  
        return exposure_val  
            


        # method for the catpuring of images from the camera
    def startCapture(self):
        i = 1
        # get the save location (path) using a file dialog
        filepath = capture_window.getFolder(self)
        print(filepath)
    # create a video object
        videoCaptureObject = cv.VideoCapture(4)

        DEFAULT_EXPOSURE_VAL = 0

        #if exposure value has not been preset
        if(self.pushButton_2.isChecked()):
            #set defined exposure value
            exposure_val = self.adjustParams()
        #else use the default  
        else:
            exposure_val = DEFAULT_EXPOSURE_VAL
        #while the startCapture button is checked 
        while (self.startButton.isChecked()):
            
            print("capture started:", i)
            #date_time = datetime.datetime.now().strftime("%m/%d/%Y:%H:%M:%S:%f")
            #time_= datetime.time
            #### gps unit code here###
            #date_time+= ".png"
            
            # read in a frame
            ret, frame = videoCaptureObject.read()
            
            #if the capture has happend properly
            if(ret == True):
                #run gradient score
                GradientScore(self,videoCaptureObject,exposure_val,frame)
                #get current gps postion
                #save the frame with data,time as the title
                name = "capture" + str(i) +".png"
                print(name)
                cv.imwrite(os.path.join(filepath,name),frame)
                i += 1
                #wait one second
                time.sleep(1)
                
            else:
                print("capture was false")
                print(filepath)
               

def NonlinearGain(self, g):
    # Nonlinear function controls the feedback gain. Simpler and faster than the ad-hoc gain adjustment presented in Shim et. al.(2018)
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
        MAX_EXPOSURE = 4
        MIN_EXPOSURE = -13
        if len(varargin) > 0:
            MAX_COUNTER = varargin[0]
        else:
            MAX_COUNTER = 50
        cap
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
                logdE = Kp * NonlinearGain(self,gamma[ptbest])
                # print(LoopCount,Exposure,Exposure+logdE)
                Exposure += logdE
                cap.set(cv.CAP_PROP_EXPOSURE, np.float64(Exposure))
                if np.abs(logdE) < 0.2:
                    bContinue = False

            if (LoopCount > MAX_COUNTER.all()):
                print("maximum number of iteration has been exceeded.")
                break
            elif(Exposure < MIN_EXPOSURE):
                print("min exposure exceeded")
                break
            elif(Exposure > MAX_EXPOSURE):
                print("max exposure exceeded")
                break

        print('GradientScore iteration count = ', LoopCount)
        return Exposure

# main code of the application#
if __name__ == "__main__":
    app = QApplication(sys.argv)
    capwindow = capture_window()
    capwindow.show()
    sys.exit(app.exec_())
