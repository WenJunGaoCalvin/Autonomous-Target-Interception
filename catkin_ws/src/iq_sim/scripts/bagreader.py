import rosbag
from std_msgs.msg import Float64, String, Int32, Bool
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from nav_msgs.msg import Odometry
import rospy
import math
import csv

def process_entry(entry,start_time):
    tsec = entry[0]
    tnanosec = entry[1]
    target_pos = entry[2]
    int_pos = entry[3]
    target_vel = entry[4]
    inter_vel = entry[5]
    inter_gimbal_yaw_rate = entry[6]
    inter_gimbal_yaw = entry[7]
    x = entry[8]
    y = entry[9]
    depth = entry[10]
    gimbal_neut_flag = entry[11]
    inter_pitch = entry[12]
    inter_yaw = entry[13]

    # process ROS data
    ts = tsec - start_time
    tns = tnanosec
    target_x = target_pos.pose.pose.position.x
    target_y = target_pos.pose.pose.position.y
    target_z = target_pos.pose.pose.position.z
    inter_x = int_pos.pose.pose.position.x
    inter_y = int_pos.pose.pose.position.y
    inter_z = int_pos.pose.pose.position.z
    target_vx = target_vel.twist.linear.x
    target_vy = target_vel.twist.linear.y
    target_vz = target_vel.twist.linear.z
    target_v = math.sqrt(target_vx**2+target_vy**2+target_vz**2)
    inter_vx = inter_vel.twist.linear.x
    inter_vy = inter_vel.twist.linear.y
    inter_vz = inter_vel.twist.linear.z
    inter_v = math.sqrt(inter_vx**2+inter_vy**2+inter_vz**2)
    inter_g_yaw_rate = inter_gimbal_yaw_rate.data
    inter_g_yaw = inter_gimbal_yaw.data
    yolo_x = x.data
    yolo_y = y.data
    yolo_depth = depth.data
    gimbal_neut = int(gimbal_neut_flag.data)
    inter_pitch2 = inter_pitch.data
    inter_yaw2 = inter_yaw.data
    
    processed = [ts,tns,target_x,target_y,target_z,inter_x,inter_y,inter_z,target_v,
                 inter_vx,inter_vy,inter_vz,inter_v,inter_g_yaw_rate,inter_g_yaw_rate,
                 inter_g_yaw,yolo_x,yolo_y,yolo_depth,gimbal_neut,inter_pitch2,inter_yaw2]
    
    
    # print(processed)
    return processed

bag = rosbag.Bag('/home/calvinwen/catkin_ws/src/iq_sim/scripts/datalog.bag')

with open("/home/calvinwen/catkin_ws/src/iq_sim/scripts/datalog.csv", mode='w') as data_file:
    data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    topics = bag.get_type_and_topic_info()[1].keys()
    num_topics = len(topics)
    # i = 0
    # csv_headers = ['seconds','nanosec']
    # for topic, msg, t in bag.read_messages(topics=topics):
    #     csv_headers.append(topic)
    #     i += 1
    #     if i == num_topics:
    #         break
    csv_headers = ['ts','tns','target_x','target_y','target_z','inter_x','inter_y','inter_z','target_v',
                'inter_vx','inter_vy','inter_vz','inter_v','inter_g_yaw_rate','inter_g_yaw_rate',
                'inter_g_yaw','yolo_x','yolo_y','yolo_depth','gimbal_neut','inter_pitch','inter_yaw']
    data_writer.writerow(csv_headers)
    j = 0
    first_entry = 1
    entry = []
    for topic, msg, t in bag.read_messages(topics=topics):
        entry.append(msg)
        j += 1
        if first_entry:
            start_time = t.secs
        if j == num_topics:
            entry = [t.secs,t.nsecs] + entry
            processed_entry = process_entry(entry,start_time)
            data_writer.writerow(processed_entry)
            j = 0
            first_entry = 0
            entry = []
bag.close()