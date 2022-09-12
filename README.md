# Image-Capture-System-ICS
This is the Image Capture Sub System of my honors project designed to run on ubuntu 20.04 (untested on 22.04 but should work) with roscore actual python scripts located at /ICS/src/ICS/scripts/

# Installation 
Install ros noetic and then clone project, run catkin_make from top ICS folder, change the value in gui.py line 15 to 
self.screen = Builder.load(<location of gui.kv on your machine>), Also make sure to run the setup.bash file in scripts folder to install extra packages

# Launch application 
To run the ICS use the command roslaunch ros_start.launch from top ICS folder to launch the gui, to exit use ctrl-c in console to fully close everything as shutdown button
in gui just shuts cameras, to restart cameras after using gui button use ctrl-c, close the gui then rerun ros_start.launch (currently working on a solution) 

