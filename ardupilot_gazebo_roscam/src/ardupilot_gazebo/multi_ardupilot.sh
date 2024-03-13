#!/bin/bash

gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone1 --console --out=tcpin:0.0.0.0:8100 -I0" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone2 --console --out=tcpin:0.0.0.0:8200 -I1" \