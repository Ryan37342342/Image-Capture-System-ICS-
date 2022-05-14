# Image Capture System (ICS)
This is the control program that I am writing for my honor project. I am working on the National Science Challenge: Bio-heritage - Urban vegetation surveillance platform project for the university. My program currently controls image capture and storage using pyqt5 and opencv2. It has two modes for taking photos one where the program will auto-adjust the exposure value, this value can be overridden in the gui. There is more to come like image recognition and an image retrival system in the future

note: this program only works on a native linux installation it WILL NOT work on a VM because of the way they handle camera inputs also this program is used with 
a intel realsense D435i camera

KNOWN BUG LIST:
-adjust value must be set before capture is started and capture must be stopped before adjust value is changed


