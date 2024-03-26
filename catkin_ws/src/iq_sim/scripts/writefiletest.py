import os, os.path
file_dir = '/home/calvinwen/catkin_ws/src/iq_sim/scripts/logs/videos/'
numfiles = len([name for name in os.listdir(file_dir) if os.path.isfile(file_dir + name)])
print(numfiles)