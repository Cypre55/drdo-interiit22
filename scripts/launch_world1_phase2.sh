#!/bin/bash

gnome-terminal -- bash -ic "git checkout added_controller_temp"
sleep 5
gnome-terminal -- bash -ic "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --console"
sleep 2
gnome-terminal -- bash -ic "roslaunch drdo_interiit22 drdo_world1.launch"
sleep 140
gnome-terminal -- bash -ic "rosrun drdo_interiit22 detector.py"
sleep 2
gnome-terminal -- bash -ic "rosrun drdo_interiit22 segmentation.py"
sleep 2
gnome-terminal -- bash -ic "rosrun drdo_interiit22 mpc.py"