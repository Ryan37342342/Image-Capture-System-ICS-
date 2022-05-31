# Image Capture System (ICS)
This is the Image Capture System that I am programing for my honor project. I am working on the National Science Challenge: Bio-heritage - Urban vegetation surveillance platform project for the university. My program currently controls image capture and storage using pyqt5 and opencv2. It has two modes for taking photos one where the program will auto-adjust the exposure value, this value can be overridden in the gui. There is more to come like image recognition and an image retrival system in the future

note: this program only works on a native linux installation (tested and running on focal 20.04) it WILL NOT work on a VM because of the way they handle camera inputs also this program is used with 
a intel realsense D435i camera

  KNOWN BUG LIST:  
  -adjust value must be set before capture is started and capture must be stopped before adjust value is changed
    -canceling select folder just prints captures to current directory instead of stopping operation
      -gps is slowing capture rate of the camera due to camera waiting on gps to call, to fix this the whole program will be changed to an ROS based python        program 


