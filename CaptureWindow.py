# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'CaptureWindow.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets

#GUI design code
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(638, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.startButton = QtWidgets.QPushButton(self.centralwidget)
        self.startButton.setGeometry(QtCore.QRect(20, 280, 111, 31))
        self.startButton.setCheckable(True)
        self.startButton.setChecked(False)
        self.startButton.setObjectName("startButton")
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QtCore.QRect(20, 180, 111, 31))
        self.pushButton_2.setObjectName("pushButton_2")
        self.setUpButton = QtWidgets.QPushButton(self.centralwidget)
        self.setUpButton.setGeometry(QtCore.QRect(20, 60, 111, 31))
        self.setUpButton.setObjectName("setUpButton")
        self.stopButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopButton.setGeometry(QtCore.QRect(20, 340, 121, 31))
        self.stopButton.setObjectName("stopButton")
        self.textAdjust = QtWidgets.QLineEdit(self.centralwidget)
        self.textAdjust.setGeometry(QtCore.QRect(20, 150, 113, 25))
        self.textAdjust.setObjectName("textAdjust")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(20, 130, 161, 17))
        self.label.setObjectName("label")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 638, 22))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionFind = QtWidgets.QAction(MainWindow)
        self.actionFind.setObjectName("actionFind")
        self.actionChange_Paramaters = QtWidgets.QAction(MainWindow)
        self.actionChange_Paramaters.setObjectName("actionChange_Paramaters")
        self.actionSave_location = QtWidgets.QAction(MainWindow)
        self.actionSave_location.setObjectName("actionSave_location")
        self.menuFile.addAction(self.actionSave_location)
        self.menubar.addAction(self.menuFile.menuAction())
        self.label.setBuddy(self.textAdjust)

        self.retranslateUi(MainWindow)
        self.startButton.clicked.connect(MainWindow.startCapture)
        self.setUpButton.clicked.connect(MainWindow.setupCamera)
        self.pushButton_2.clicked.connect(MainWindow.adjustParams)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.startButton.setText(_translate("MainWindow", "Start Capture"))
        self.pushButton_2.setText(_translate("MainWindow", "Adjust Camera"))
        self.setUpButton.setText(_translate("MainWindow", "run setup"))
        self.stopButton.setText(_translate("MainWindow", "Stop Capture"))
        self.label.setText(_translate("MainWindow", "Enter custom exposure value"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.actionFind.setText(_translate("MainWindow", "Find"))
        self.actionChange_Paramaters.setText(_translate("MainWindow", "Change Paramaters"))
        self.actionSave_location.setText(_translate("MainWindow", "Save location"))