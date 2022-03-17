#!/bin/bash

gnome-terminal -- bash -ic "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --console"
sleep 2
gnome-terminal -- bash -ic "roslaunch drdo_interiit22 drdo_world1.launch"

sleep 140
gnome-terminal -- bash -ic "rosrun drdo_interiit22 segmentation.py"
sleep 3
gnome-terminal -- bash -ic "rosrun drdo_interiit22 detector.py"
sleep 2
gnome-terminal -- bash -ic "cd ~/interiit_new_ws/src/drdo_interiit22/src/Controller"
gnome-terminal -- bash -ic "python high_level_drone_control.py"
sleep 4
gnome-terminal -- bash -ic "rosrun drdo_interiit22 mpc.py"


