# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'CaptureWindow.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(476, 346)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.adjustCamera = QtWidgets.QPushButton(self.centralwidget)
        self.adjustCamera.setGeometry(QtCore.QRect(10, 130, 131, 31))
        self.adjustCamera.setCheckable(True)
        self.adjustCamera.setAutoRepeatDelay(0)
        self.adjustCamera.setAutoRepeatInterval(0)
        self.adjustCamera.setObjectName("adjustCamera")
        self.setUpButton = QtWidgets.QPushButton(self.centralwidget)
        self.setUpButton.setGeometry(QtCore.QRect(10, 30, 141, 31))
        self.setUpButton.setObjectName("setUpButton")
        self.startButton = QtWidgets.QPushButton(self.centralwidget)
        self.startButton.setGeometry(QtCore.QRect(10, 180, 131, 31))
        self.startButton.setCheckable(True)
        self.startButton.setAutoRepeatDelay(0)
        self.startButton.setAutoRepeatInterval(0)
        self.startButton.setObjectName("startButton")
        self.textAdjust = QtWidgets.QLineEdit(self.centralwidget)
        self.textAdjust.setGeometry(QtCore.QRect(10, 100, 113, 25))
        self.textAdjust.setObjectName("textAdjust")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(10, 76, 161, 21))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(280, 80, 161, 21))
        self.label_2.setObjectName("label_2")
        self.timingAdjust = QtWidgets.QLineEdit(self.centralwidget)
        self.timingAdjust.setGeometry(QtCore.QRect(280, 100, 113, 25))
        self.timingAdjust.setObjectName("timingAdjust")
        self.timingButton = QtWidgets.QPushButton(self.centralwidget)
        self.timingButton.setGeometry(QtCore.QRect(280, 130, 141, 31))
        self.timingButton.setCheckable(True)
        self.timingButton.setAutoRepeatDelay(0)
        self.timingButton.setAutoRepeatInterval(0)
        self.timingButton.setObjectName("timingButton")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.startButton.clicked.connect(MainWindow.startCapture)
        self.adjustCamera.clicked.connect(MainWindow.adjustParams)
        self.setUpButton.clicked.connect(MainWindow.setUp)
        self.timingButton.clicked.connect(MainWindow.adjustTiming)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.adjustCamera.setText(_translate("MainWindow", "AdjustCamera"))
        self.setUpButton.setText(_translate("MainWindow", "run set up"))
        self.startButton.setText(_translate("MainWindow", "Start Capture"))
        self.label.setText(_translate("MainWindow", "Adjust exposure"))
        self.label_2.setText(_translate("MainWindow", "Adjust timing"))
        self.timingButton.setText(_translate("MainWindow", "Set time"))
