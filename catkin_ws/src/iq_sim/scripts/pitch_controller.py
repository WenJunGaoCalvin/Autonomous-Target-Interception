#!/usr/bin/env python3

from std_msgs.msg import Float64, String, Int32, Bool
from nav_msgs.msg import Odometry
import rospy
import math

class PitchController:
    def __init__(self):
        # data to log
        self.interceptor_pose_g = Odometry()
        # calculated variables
        self.interceptor_pitch = Float64()
        self.interceptor_yaw = Float64()

        # ros subscribers to support data logging
        self.interceptorPos = rospy.Subscriber(
            name="/interceptor/mavros/global_position/local",
            data_class=Odometry,
            queue_size=10,
            callback=self.interceptor_pose_cb,
        )

        self.pitch_pub = rospy.Publisher(
            name="/ros_pitch",
            data_class=Float64,
            queue_size=10,
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
    
    def interceptor_pose_cb(self, message):
        self.interceptor_pose_g = message
        self.updateEuler()

rospy.init_node("pitch_controller")
rate = rospy.Rate(60)

interceptor = PitchController()
while not rospy.is_shutdown():
    inter_pitch  = interceptor.interceptor_pitch
    interceptor.pitch_pub.publish(inter_pitch)
    rate.sleep()