#!/bin/bash

gnome-terminal -- bash -ic "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --console"
sleep 2
gnome-terminal -- bash -ic "roslaunch drdo_interiit22 drdo_world2_phase2.launch"


