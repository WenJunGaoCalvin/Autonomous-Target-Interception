#!/usr/bin/env python3

import rosbag
from std_msgs.msg import Float64, String, Int32, Bool
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from nav_msgs.msg import Odometry
import rospy
import math

class logger_api:
    def __init__(self):
        # data to log
        self.target_pose_g = Odometry()
        self.interceptor_pose_g = Odometry()
        self.target_vel = TwistStamped()
        self.interceptor_vel = TwistStamped()
        self.current_gimbal_pitch = Float64()
        self.current_gimbal_yaw = Float64()
        self.target_img_coord_x = Int32()
        self.target_img_coord_y = Int32()
        self.target_img_coord_depth = Float64()
        self.gimbal_neut_flag = Bool()
        self.record_flag = Bool()

        # calculated variables
        self.interceptor_pitch = Float64()
        self.interceptor_yaw = Float64()

        # ros subscribers to support data logging
        self.targetPos = rospy.Subscriber(
            name="/target/mavros/global_position/local",
            data_class=Odometry,
            queue_size=10,
            callback=self.target_pose_cb,
        )

        self.interceptorPos = rospy.Subscriber(
            name="/interceptor/mavros/global_position/local",
            data_class=Odometry,
            queue_size=10,
            callback=self.interceptor_pose_cb,
        )

        self.targetVel = rospy.Subscriber(
            name="/target/mavros/local_position/velocity_local",
            data_class=TwistStamped,
            queue_size=10,
            callback=self.target_vel_cb,
        )

        self.interceptorVel = rospy.Subscriber(
            name="/interceptor/mavros/local_position/velocity_local",
            data_class=TwistStamped,
            queue_size=10,
            callback=self.interceptor_vel_cb,
        )

        self.gimbalPitch = rospy.Subscriber(
            name="/ros_gimbal_pitch_status",
            data_class=String,
            queue_size=10,
            callback=self.gimbal_pitch_cb,
        )

        self.gimbalYaw = rospy.Subscriber(
            name="/ros_gimbal_yaw_status",
            data_class=String,
            queue_size=10,
            callback=self.gimbal_yaw_cb,
        )

        self.targetYoloX = rospy.Subscriber(
            name="/yolo_detect_coord_x",
            data_class=Int32,
            queue_size=10,
            callback=self.target_yolo_x_cb,
        )

        self.targetYoloY = rospy.Subscriber(
            name="/yolo_detect_coord_y",
            data_class=Int32,
            queue_size=10,
            callback=self.target_yolo_y_cb,
        )

        self.targetYoloZ = rospy.Subscriber(
            name="/yolo_detect_coord_z",
            data_class=Float64,
            queue_size=10,
            callback=self.target_yolo_z_cb,
        )

        self.gimbalNeut = rospy.Subscriber(
            name="/gimbal_neutralisation_status",
            data_class=Bool,
            queue_size=10,
            callback=self.gimbal_neut_cb,
        )

        self.dataLog = rospy.Subscriber(
            name="/log_data_flag",
            data_class=Bool,
            queue_size=10,
            callback=self.data_log_cb,
        )

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians        

    def updateEuler(self):
        q0, q1, q2, q3 = (
        self.interceptor_pose_g.pose.pose.orientation.w,
        self.interceptor_pose_g.pose.pose.orientation.x,
        self.interceptor_pose_g.pose.pose.orientation.y,
        self.interceptor_pose_g.pose.pose.orientation.z,
        )
        pitch,roll,yaw = self.euler_from_quaternion(q1, q2, q3, q0)
        self.interceptor_pitch.data = pitch
        self.interceptor_yaw.data = yaw

    # callback functions for ros subscribers
    def target_pose_cb(self, message):
        self.target_pose_g = message
    
    def interceptor_pose_cb(self, message):
        self.interceptor_pose_g = message
        self.updateEuler()
    
    def target_vel_cb(self, message):
        self.target_vel = message
    
    def interceptor_vel_cb(self, message):
        self.interceptor_vel = message
    
    def gimbal_pitch_cb(self, message):
        self.current_gimbal_pitch.data = float(message.data)
    
    def gimbal_yaw_cb(self, message):
        self.current_gimbal_yaw.data = float(message.data)
                                            
    def target_yolo_x_cb(self, message):
        self.target_img_coord_x = message
    
    def target_yolo_y_cb(self, message):
        self.target_img_coord_y = message

    def target_yolo_z_cb(self, message):
        self.target_img_coord_depth = message
    
    def gimbal_neut_cb(self, message):
        self.gimbal_neut_flag = message
    
    def data_log_cb(self, message):
        self.record_flag = message

rospy.init_node("datalogger")
rate = rospy.Rate(60)

log_inactive = 1
bag = rosbag.Bag('/home/calvinwen/catkin_ws/src/iq_sim/scripts/datalog.bag', 'w')
# create 1 bag for unified timer object
datalogger = logger_api()
while not rospy.is_shutdown():
    if datalogger.record_flag.data:
        if log_inactive:
            rospy.loginfo("Data logging begin")
            log_inactive = 0
        bag.write('target_pos',datalogger.target_pose_g)
        bag.write('interceptor_pos',datalogger.interceptor_pose_g)
        bag.write('target_vel',datalogger.target_vel)
        bag.write('interceptor_vel',datalogger.interceptor_vel)
        bag.write('interceptor_gimbal_yaw_rate',datalogger.current_gimbal_pitch)
        bag.write('interceptor_gimbal_yaw',datalogger.current_gimbal_yaw)
        bag.write('target_img_coord_x',datalogger.target_img_coord_x)
        bag.write('target_img_coord_y',datalogger.target_img_coord_y)
        bag.write('target_img_coord_depth',datalogger.target_img_coord_depth)
        bag.write('gimbal_neut',datalogger.gimbal_neut_flag)
        bag.write('interceptor_pitch',datalogger.interceptor_pitch)
        bag.write('interceptor_yaw',datalogger.interceptor_yaw)
    elif not datalogger.record_flag.data and not log_inactive:
        topics = bag.get_type_and_topic_info()[1].keys()
        for topic in topics:
            print(topic)
        # types = []
        # for val in bag.get_type_and_topic_info()[1].values():
        #     print(val)
            # types.append(val[0])
        bag.close()
        rospy.loginfo("Data logging finished")
        log_inactive = 1
    rate.sleep()