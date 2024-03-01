# Autonomous-Target-Interception
Development of an autonomous target interception package that allows a drone to move towards a target using vision based navigation

### Motivation
With the increasing threat of hostile drone swarms, there needs to be a solution to neutralise these threats. One of such solutions is the use of a group of drone interceptors. These interceptors require a target interception software package that will guide them towards their assigned targets before they can perform neutralisation action.

### Technology Stack
Sensor: Azure Kinect DK Stereo Camera
Object Detection and Classification: YOLOv8m
Guidance Law: Image-Based Visual Servoing
Velocity Controller: PID
Flight Controller: ArduPilot

### Simulation
Simulator: Gazebo
Simulated drone: Iris SITL

### Current Works
Implementing gimbal control to enable more robust target locking.

