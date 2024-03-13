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

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_vel_pub = rospy.Publisher('mavros/setpoint_raw/local',PositionTarget, queue_size = 10)
    
    # yaw_rate_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',PositionTarget, queue_size = 10)

    # local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)




    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # pose = PoseStamped()

    # pose.pose.position.x = 0
    # pose.pose.position.y = 0
    # pose.pose.position.z = 2
    
    # setting velocity in body frame
    vel_msg = PositionTarget()
    speed = 1
    radius = 5
    omega = speed/radius
    vel_msg.type_mask = 0b010111000111
    vel_msg.coordinate_frame = 8
    vel_msg.velocity.x = 5
    vel_msg.velocity.y = 0
    #vel_msg.velocity.z = 0
    #vel_msg.yaw = 0
    vel_msg.yaw_rate = omega


    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        # local_pos_pub.publish(pose)
        local_vel_pub.publish(vel_msg)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'GUIDED'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "GUIDED" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("GUIDED enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
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

                last_req = rospy.Time.now()

        # local_pos_pub.publish(pose)
        local_vel_pub.publish(vel_msg)
        #rospy.loginfo("Velocity updating")

        rate.sleep()

