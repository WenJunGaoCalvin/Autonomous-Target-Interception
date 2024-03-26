import rosbag
from std_msgs.msg import Float64, String, Int32, Bool
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from nav_msgs.msg import Odometry
import rospy
import math
import csv
import os, os.path

def process_entry(entry,start_time):
    tsec = entry[0]
    tnanosec = entry[1]
    target_pos = entry[2]
    int_pos = entry[3]
    target_vel = entry[4]
    inter_vel = entry[5]
    x = entry[6]
    y = entry[7]
    depth = entry[8]
    inter_pitch = entry[9]
    inter_yaw = entry[10]

    # process ROS data
    ts = tsec - start_time
    tns = tnanosec
    t = float(str(ts) + '.' + str(tns)[:3])
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
    yolo_x = x.data
    yolo_y = y.data
    yolo_depth = depth.data
    inter_pitch2 = inter_pitch.data
    inter_yaw2 = inter_yaw.data
    
    processed = [t,target_x,target_y,target_z,inter_x,inter_y,inter_z,target_v,
                 inter_vx,inter_vy,inter_vz,inter_v,
                 yolo_x,yolo_y,yolo_depth,inter_pitch2,inter_yaw2]
    
    
    # print(processed)
    return processed

file_dir = '/home/calvinwen/catkin_ws/src/iq_sim/scripts/logs/no_gimbal/'
filename = '19_datalog.bag' # to change
bag = rosbag.Bag(file_dir + "rosbags/" + filename)
num = len([name for name in os.listdir(file_dir + "csv") if os.path.isfile(file_dir + "csv/" + name)])
with open(file_dir + "csv/" + str(num) + "_datalog.csv", mode='w') as data_file:
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
    csv_headers = ['t','target_x','target_y','target_z','inter_x','inter_y','inter_z','target_v',
                'inter_vx','inter_vy','inter_vz','inter_v',
                'yolo_x','yolo_y','yolo_depth','inter_pitch','inter_yaw']
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