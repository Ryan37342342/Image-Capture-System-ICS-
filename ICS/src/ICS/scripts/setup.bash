#!/bin/bash

source ~/PycharmProjects/Image-Capture-System-ICS-/ICS/devel/setup.bash
sudo apt install -y python3-pip
sudo apt-get install -y python3-rospy
pip3 install roslibpy
pip3 install kivymd
pip3 install pyserial
pip3 install PyYAML
# this is the python wrapper for the basler cameras you may have to install from source
pip3 install pypylon
pip3 install sparkfun-ublox-gps
pip3 install spidev
sudo apt install python3-cv-bridge
sudo chmod 666 /dev/ttyACM0
