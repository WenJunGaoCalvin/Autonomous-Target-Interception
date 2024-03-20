#!/usr/bin/env python3


"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""


import rospy
# import move_circle_server
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from iq_gnc.py_gnc_functions import *
import argparse
import math

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', type=str, default="/")
    args = rospy.myargv()
    args = parser.parse_args(args[1:])
    ns = args.ns
    
    # Initialise ROS node
    rospy.init_node("offb_node_py")

    # Create target as an object for the API.
    target = gnc_api(ns)
    # Wait for FCU connection.
    target.wait4connect()
    # Wait for the mode to be switched.
    target.wait4start()
    # Request takeoff with an altitude of 10m.
    # target.takeoff(10)

    # Specify control loop rate. Setpoint publishing MUST be faster than 2Hz
    loop_freq = 20 #Hz
    rate = rospy.Rate(loop_freq)

    # setting velocity in body frame
    circle_vel_msg = PositionTarget()
    speed = 0 #1
    radius = 5
    omega = speed/radius
    circle_vel_msg.type_mask = 0b010111000111
    circle_vel_msg.coordinate_frame = 8
    circle_vel_msg.velocity.x = 0 #5
    circle_vel_msg.velocity.y = 0
    #vel_msg.velocity.z = 0
    #vel_msg.yaw = 0
    circle_vel_msg.yaw_rate = omega

    osc_vel_msg = PositionTarget()
    osc_vel_msg.type_mask = 0b010111000111
    osc_vel_msg.coordinate_frame = 8
    osc_vel_msg.velocity.x = 1 #5
    i = 0
    period = 10
    osc_freq = 1/period
    amp = 1


    # # Send a few setpoints before starting
    # for i in range(100):
    #     if(rospy.is_shutdown()):
    #         break

    #     # local_pos_pub.publish(pose)
    #     target.local_vel_pub.publish(vel_msg)
    #     rate.sleep()

    # Set mode to GUIDED
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'GUIDED'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "GUIDED" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(target.set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("GUIDED enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(target.arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                    # rospy.sleep(5)

                    # print ("\nTaking off")

                    # rospy.wait_for_service('/mavros/cmd/takeoff')
                    # takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

                    # # set request object
                    # req = CommandTOLRequest()
                    # req.yaw = 0.0
                    # req.latitude = 0.0
                    # req.longitude = 0.0
                    # req.altitude = 50.0
                    # req.min_pitch = 0.0

                    # try:
                    #     response = takeoff_cl.call(req)
                    #     rospy.loginfo(response)
                    # except rospy.ServiceException as e:
                    #     print("Takeoff failed: %s" %e)
                    # target.takeoff(10)

                last_req = rospy.Time.now()

        # local_pos_pub.publish(pose)
        if i > (1/osc_freq):
            i = 0
        omega = 2*3.14159*osc_freq
        y_rate = omega*amp*math.cos(omega*i)
        osc_vel_msg.velocity.y = y_rate
        i+= 1/loop_freq

        # chooose flight profile

        # target.local_vel_pub.publish(osc_vel_msg)
        target.local_vel_pub.publish(osc_vel_msg)

        rate.sleep()

