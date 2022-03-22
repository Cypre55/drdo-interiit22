#!/bin/bash

gnome-terminal -- bash -ic "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --console"
sleep 2
gnome-terminal -- bash -ic "roslaunch drdo_interiit22 drdo_world2_phase1.launch"
sleep 30
gnome-terminal -- bash -ic "rosrun drdo_interiit22 graph_plannerr_new.py"
sleep 25
gnome-terminal -- bash -ic "rosrun drdo_interiit22 segmentation_w2.py"