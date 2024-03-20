# Autonomous-Target-Interception
Development of an autonomous target interception package that allows a drone to move towards a target using vision based navigation

### Motivation
With the increasing threat of hostile drone swarms, there needs to be a solution to neutralise these threats. One of such solutions is the use of a group of drone interceptors. These interceptors require a target interception software package that will guide them towards their assigned targets before they can perform neutralisation action.

### Technology Stack
Sensor: Azure Kinect DK Stereo Camera<br /> 
Object Detection and Classification: YOLOv8m<br /> 
Guidance Law: Image-Based Visual Servoing<br /> 
Velocity Controller: PID<br /> 
Flight Controller: ArduPilot

### Simulation
Simulator: Gazebo<br /> 
Simulated drone: Iris SITL

### Current Works
Implementing gimbal control to enable more robust target locking.

### Commands
roslaunch ardupilot_gazebo multi_iris_with_roscam.launch<br /> 

sim_vehicle.py -v ArduCopter -f gazebo-drone1 --console -I0 --out=tcpin:0.0.0.0:8100<br /> 
sim_vehicle.py -v ArduCopter -f gazebo-drone2 --console -I1 --out=tcpin:0.0.0.0:8200 -M<br /> 

roslaunch iq_sim multi_apm.launch<br /> 
roslaunch iq_sim multi_cuav_gnc.launch<br /> 
roslaunch iq_sim cuav_interceptor_gnc.launch<br /> 
roslaunch iq_sim cuav_interceptor_gnc_gimbal.launch
