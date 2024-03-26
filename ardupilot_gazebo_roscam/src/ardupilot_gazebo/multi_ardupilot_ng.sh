#!/bin/bash

gnome-terminal \
 --tab -e "roslaunch ardupilot_gazebo multi_iris_with_roscam.launch" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone1 --map --console -I0 --out=tcpin:0.0.0.0:8100" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone2 --console -I1 --out=tcpin:0.0.0.0:8200" \
 --tab -e "rqt" \
#  --tab -e "roslaunch iq_sim multi_apm.launch" \