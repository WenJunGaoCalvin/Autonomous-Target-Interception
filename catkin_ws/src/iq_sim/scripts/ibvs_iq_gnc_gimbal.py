#!/usr/bin/env python3


"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""


import rospy
import torch
import os, sys
from pathlib import Path
from utils.general import check_requirements
# import move_circle_server
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import State, PositionTarget, MountControl
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest, MountConfigureRequest
from sensor_msgs.msg import Image
from iq_gnc.py_gnc_functions import *
import argparse
import cv2
import numpy as np

import argparse
import time, os, sys
from pathlib import Path

from std_msgs.msg import String, Float64, Bool, Int32
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import actionlib
from cv_bridge import CvBridge
import csv

import torch.backends.cudnn as cudnn
from numpy import random
import pyrealsense2 as rs
from ultralytics import YOLO

from utils.dataloaders import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, non_max_suppression, apply_classifier, scale_boxes, \
    xyxy2xywh, strip_optimizer, set_logging, increment_path, clip_boxes
from utils.plots import save_one_box, plot_images
from utils.torch_utils import select_device, reshape_classifier_output, time_sync

from simple_pid import PID
import math
import rosbag
import time
import csv
import os, os.path

file_dir = '/home/calvinwen/catkin_ws/src/iq_sim/scripts/logs/gimbal/videos/'
time_file_dir = '/home/calvinwen/catkin_ws/src/iq_sim/scripts/logs/gimbal/time/'

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return img, ratio, (dw, dh)

def plot_one_box(x, im, color=None, label=None, line_thickness=None):
    # Plots one bounding box on image img
    tl = line_thickness or round(0.002 * (im.shape[0] + im.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(im, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(im, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(im, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

def process_objects_on_image(results):
    """
    Function receives an image,
    passes it through YOLOv8 neural network
    and returns an array of detected objects
    and their bounding boxes
    :param buf: Input image file stream
    :return: Array of bounding boxes in format [[x1,y1,x2,y2,probability,class],..]
    """
    # result = results[0]
    output = []
    for result in results:
        output.append([result.boxes.xyxy, result.boxes.conf,result.boxes.cls])

    #     print(box)
    #     x1, y1, x2, y2 = [
    #         round(x) for x in box.xyxy[0].tolist()
    #     ]
    #     class_id = box.cls[0].item()
    #     prob = round(box.conf[0].item(), 2)
    #     output.append([
    #         x1, y1, x2, y2, result.names[class_id], prob
    #     ])
    return output

def get_depth(depth_img, d1,d2):
    a = 10
    z_arr = []
    height = len(depth_img)
    width= len(depth_img[0])
    for i in range(d1-2*a,d1):
        for j in range(d2-a,d2+a):
            if (0<=i<height) and (0<=j<width): 
                if not np.isnan(depth_img[j][i]):
                    zDepth = depth_img[j][i]
                    z_arr.append(zDepth)
    if not z_arr:
        return -0.01
    zDepth = np.mean(z_arr)
    return zDepth

def get_kinect_distance_img(img,interceptorObj):
    bridge=CvBridge()
    depth_image = bridge.imgmsg_to_cv2(img, desired_encoding="32FC1")
    interceptorObj.update_depth_image(depth_image)

def callback(image_msg,interceptorObj):
    # rospy.loginfo("Callback")
    # Convert the image to OpenCV image 
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    interceptorObj.update_colour_img(cv_image)

def kinect_detect(opt,model,names,colors,device, interceptorObj,video):
    try:
        yolo_pred(interceptorObj.colour_img,interceptorObj.depth_image, model,names,colors,device,opt,interceptorObj,video)
    except:
        print("image input error")
    else:
        yolo_pred(interceptorObj.colour_img,interceptorObj.depth_image, model,names,colors,device,opt,interceptorObj,video)

def yolo_pred(color_image,depth,model,names,colors,device,opt,interceptorObj,video):
    if type(depth)==int:
        return

    im0s = color_image
    img = letterbox(im0s)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device)
    img = img.float()
    # img = img.half() if device.type != 'cpu' else img.float()  # uint8 to fp16/32 
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    # Inference
    t1 = time_sync()
    pred = model.predict(img, augment=opt.augment, agnostic_nms=True)[0]

    # Process detections
        
    # Process results list
    img_info = process_objects_on_image(pred)
    if not img_info:
        im0 = color_image
        d1,d2,zDepth = -1,-1,-1
    for i in img_info:  # detections per image
        # i: [x1,y1,x2,y2,probability,class]
        im0 = im0s
        # gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        if len(i):
            # Rescale boxes from img_size to im0 size
            i[0] = scale_boxes(img.shape[2:], i[0], im0.shape).round()

            # Write results
            xyxy = i[0][0]
            conf = float(i[1])
            cls = int(i[2])                    
            if not cls:               
                label = f'{names[int(cls)]} {conf:.2f}'
                plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)
                #############################################
                d1, d2 = int((int(xyxy[0])+int(xyxy[2]))/2), int((int(xyxy[1])+int(xyxy[3]))/2)
                if type(depth)==int:
                    rospy.loginfo("No distance reading")
                    zDepth = -1
                else:
                    zDepth = get_depth(depth,d1,d2)
                tl = 3 #line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
                tf = max(tl - 1, 1)  # font thickness
                cv2.putText(im0, str(round((zDepth),2))+" m", (d1, d2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
                print("Print PID inputs: d1 d2 zDepth")
                print(d1,d2,zDepth)
                #############################################

    interceptorObj.updatebbox(im0)
    interceptorObj.update_xyz(d1,d2,zDepth)
    # print("Size")
    # print(im0.shape)
    video.write(im0)
    # cv2.imshow("Stream", im0)
    cv2.waitKey(1)

def yolo_init(device):
    # Initialize
    set_logging()
    
    half = device.type != 'cpu'  # half precision only supported on CUDA
    model = YOLO('/home/calvinwen/catkin_ws/src/iq_sim/scripts/best.pt')
    if half:
        model.half()  # to FP16

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]    
    return model, names, colors

class Interceptor ():
    def __init__(self,):
        self.depth_image = 0
        self.colour_img = 0
        self.bbox_img = 0
        self.x = -1
        self.y= -1
        self.z= -1
    
    def update_xyz(self,x,y,z):
        self.x = x
        self.y= y
        self.z= z
    
    def update_depth_image(self,depth_image):
        self.depth_image = depth_image
    
    def update_colour_img(self, colour):
        self.colour_img = colour
    
    def updatebbox(self,im0):
        self.bbox_img = im0

def ibvs_init():
    Ts = 0.1
    Kx = np.array([0.4,0.002, 0]) #Kmax = 1, f0 = 1/21
    Ky = np.array([1,0, 0.0001])/480
    Kyaw = np.array([1,0.2,0.1])
    # Kx = np.array([0,0, 0.000])
    # Ky = np.array([0,0.0, 0.0])
    # Kyaw = np.array([0,0.0, 0.0])
    KgimbalYaw = np.array([0.002,0.0,0])
    
    setpoints = [320,240,5,0] #image is 640 by 480
    xPID = PID(Kx[0],Kx[1],Kx[2],setpoints[2]) # forward PID
    xPID.output_limits = (-5,5)
    xPID.sample_time = Ts
    yPID = PID(Ky[0],Ky[1],Ky[2],setpoints[1]) # up PID
    yPID.output_limits = (-10,10)
    yPID.sample_time = Ts
    quadYawPID = PID(Kyaw[0],Kyaw[1],Kyaw[2],setpoints[3]) # Drone yaw PID
    quadYawPID.output_limits = (-3.14,3.14) #do not change limits beyond -3.14 and 3.14
    quadYawPID.sample_time = Ts
    gimbalYawPID = PID(KgimbalYaw[0],KgimbalYaw[1],KgimbalYaw[2],setpoints[0])
    gimbalYawPID.output_limits = (-0.5,0.5) #do not change limits beyond -3.14 and 3.14
    gimbalYawPID.sample_time = Ts
    return xPID, yPID, quadYawPID, gimbalYawPID

def ibvs_getVel(x, y,z, xPID, yPID, gimbalYawPID):
    vx = -xPID(z)
    vy = yPID(y)
    gimbal_yaw_rate = float(gimbalYawPID(x))
    print("PID Outputs: Vx Vy GimbalYawRate")
    print(vx,vy,gimbal_yaw_rate)
    return vx, vy, gimbal_yaw_rate

def ibvs_getYaw(yaw,yawPID):
    yaw_rate = yawPID(yaw)
    return yaw_rate

def neutralise_yaw(drone,quadYawPID,gimbal_neut_flag,gimbal_neut_status_pub,vx,vy,kinect_params):
    # publish a vel_msg to yaw exactly by how much gimbal yaw is
    rospy.loginfo("Neutralising gimbal")
    gimbal_neut_flag.data = 1
    gimbal_neut_status_pub.publish(gimbal_neut_flag)

    vel_msg = PositionTarget()
    vel_msg.type_mask = 0b010111000111
    vel_msg.coordinate_frame = 8
    inner_threshold = 0.06

    opt,model,names,colors,device,interceptorObj,video = kinect_params

    threshold = 0.349066 # 20 deg
    yaw_rate_lower = 0.3
    yaw_rate_upper = 1

    while abs(drone.current_gimbal_yaw)>inner_threshold: # around 3-5 deg from neutral
        # kinect_detect(opt,model,names,colors,device,interceptorObj,video)
        x = interceptorObj.x
        if x >= 0:
            vel_msg.velocity.z = 0
            vel_msg.velocity.x = vx
        elif x<0 and abs(vx) > 0.07:
            vel_msg.velocity.z = 0
            vel_msg.velocity.x = 0

        # gimbal_yaw_rate = quadYawPID(drone.current_gimbal_yaw)

        if drone.current_gimbal_yaw > threshold:
            # gimbal_yaw_rate = quadYawPID(drone.current_gimbal_yaw)
            gimbal_yaw_rate = -1*yaw_rate_upper
        elif inner_threshold < drone.current_gimbal_yaw < threshold:
            gimbal_yaw_rate = -1*yaw_rate_lower
        elif -1*inner_threshold > drone.current_gimbal_yaw > -1*threshold:
            gimbal_yaw_rate = yaw_rate_lower
        elif drone.current_gimbal_yaw < -1*threshold:
            gimbal_yaw_rate = yaw_rate_upper
        elif abs(drone.current_gimbal_yaw)>inner_threshold:
            gimbal_yaw_rate = 0
        quad_yaw_rate = -1*gimbal_yaw_rate
        # quadcopter action to neutralise gimbal
        vel_msg.yaw_rate = quad_yaw_rate
        drone.local_vel_pub.publish(vel_msg)
        # neutralise gimbal using negaitive of yaw rate
        drone.gimbal_yaw_rate_pub.publish(gimbal_yaw_rate)
    
    # stop yawing when within range
    vel_msg.yaw_rate = 0
    gimbal_yaw_rate = 0   
    drone.gimbal_yaw_rate_pub.publish(gimbal_yaw_rate)
    drone.local_vel_pub.publish(vel_msg)
    rospy.loginfo("Gimbal neutralised")
    gimbal_neut_flag.data = 0
    gimbal_neut_status_pub.publish(gimbal_neut_flag)

def euler_from_quaternion(x, y, z, w):
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

def control_pitch(drone):
    roll, pitch, yaw = euler_from_quaternion(drone.current_pose_g.pose.pose.orientation.x,
                                             drone.current_pose_g.pose.pose.orientation.y,
                                             drone.current_pose_g.pose.pose.orientation.z,
                                             drone.current_pose_g.pose.pose.orientation.w)
    pitch_topic = "/ros_pitch"

    #send pitch to pitch topic
    pitch_cmd = float(-1*pitch)
    # rospy.loginfo(pitch)
    drone.gimbal_pitch_pub.publish(pitch_cmd)
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', type=str, default="/")
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'best.pt', help='model.pt path(s)')
    # parser.add_argument('--source', type=str, default='data/images', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')

    # opt = parser.parse_args()
    opt = rospy.myargv()
    opt = parser.parse_args(opt[1:])
    ns = opt.ns

    # Initialise
    rospy.init_node("interceptor_gnc")
    # bag = rosbag.Bag('path_taken.bag','w')

    # Specify control loop rate. Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(60)
    device = select_device(opt.device)
    model,names,colors = yolo_init(device)
    
    interceptorObj = Interceptor()

    rospy.Subscriber('interceptor/camera_ir/camera/depth/image_raw',Image,get_kinect_distance_img, interceptorObj, queue_size = 1)
    rospy.Subscriber("interceptor/camera_ir/camera/color/image_raw", Image, callback, interceptorObj, queue_size = 1, buff_size = 16777216)
    
    yolo_detect_x_pub = rospy.Publisher(
            name="/yolo_detect_coord_x",
            data_class=Int32,
            queue_size = 10
        )
    yolo_detect_y_pub = rospy.Publisher(
            name="/yolo_detect_coord_y",
            data_class=Int32,
            queue_size = 10
        )
    yolo_detect_z_pub = rospy.Publisher(
            name="/yolo_detect_coord_z",
            data_class=Float64,
            queue_size = 10
        )
    gimbal_neut_status_pub = rospy.Publisher(
            name="/gimbal_neutralisation_status",
            data_class=Bool,
            queue_size = 10)
    log_flag_pub = rospy.Publisher( # to be controlled externally
            name="/log_data_flag",
            data_class=Bool,
            queue_size = 10)

    # other initialisations of ros data structures
    gimbal_neut_flag = Bool()
    ros_yolo_x = Int32()
    ros_yolo_y = Int32()
    ros_yolo_depth = Float64()

    xPID, yPID, quadYawPID, gimbalYawPID = ibvs_init()

    # Video Capture code
    frame_width = 640
    frame_height = 480
   
    size = (frame_width, frame_height)

    num = len([name for name in os.listdir(file_dir) if os.path.isfile(file_dir + name)])
    timenum = len([name for name in os.listdir(time_file_dir) if os.path.isfile(time_file_dir + name)])

    video = cv2.VideoWriter(file_dir + str(num) +'_filename.avi',  
                         cv2.VideoWriter_fourcc(*'MJPG'), 
                         1, size) 
    
    # Create interceptor as an object for the API.
    interceptor_gnc = gnc_api(ns)
    # Wait for FCU connection.
    interceptor_gnc.wait4connect()
    # Wait for the mode to be switched.
    interceptor_gnc.wait4start()

    

    # setting velocity in body frame
    vel_msg = PositionTarget()
    vel_msg.type_mask = 0b010111000111
    vel_msg.coordinate_frame = 8
    vel_msg.velocity.x, vel_msg.velocity.z, vel_msg.yaw_rate = 0,0,0

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

    first_loop = 1
    last_known_vx = 0
    last_known_vy = 0

    kinect_params = [opt,model,names,colors,device,interceptorObj,video]

    while not rospy.is_shutdown():
        tic = time.perf_counter()
        if(current_state.mode != "GUIDED" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(interceptor_gnc.set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("GUIDED enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(interceptor_gnc.arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        with torch.no_grad():
            kinect_detect(opt,model,names,colors,device,interceptorObj,video)
            im0 = interceptorObj.colour_img
            x = interceptorObj.x
            y = interceptorObj.y
            z = interceptorObj.z

            ros_yolo_x.data = x
            ros_yolo_y.data = y
            ros_yolo_depth.data = z

            yolo_detect_x_pub.publish(ros_yolo_x)
            yolo_detect_y_pub.publish(ros_yolo_y)
            yolo_detect_z_pub.publish(ros_yolo_depth)

            if type(im0) == int:
                continue
            yaw = interceptor_gnc.current_gimbal_yaw
            
            control_pitch(interceptor_gnc)
            # if abs(yaw)>0.087:
            if abs(yaw)>0.06:
                # stop chase & neutralise gimbal joint yaw between -10 to 10 deg
                neutralise_yaw(interceptor_gnc, quadYawPID,gimbal_neut_flag,gimbal_neut_status_pub,last_known_vx,last_known_vy,kinect_params)
            else:

                if x > 0:
                    # if a coord is available, update yaw and vertical velocity
                    vx, vy, gimbal_yaw_rate = ibvs_getVel(x,y,z, xPID, yPID, gimbalYawPID)
                    # yaw_rate = ibvs_getYaw(yaw,quadYawPID)
                    interceptor_gnc.gimbal_yaw_rate_pub.publish(gimbal_yaw_rate)
                    vel_msg.velocity.z = vy
                    last_known_vy = vy
                    vel_msg.yaw_rate = 0

                    if z > 0:
                        # if a depth reading is available, update forward velocity
                        vel_msg.velocity.x = vx
                        last_known_vx = vx
                    elif z < 0 and last_known_vx:
                        vel_msg.velocity.x = last_known_vx
                    else:
                        # forward creep   
                        vel_msg.velocity.x = 1

                else:
                    # if no detection, spin to find target
                    # neutralise_yaw(interceptor_gnc,quadYawPID,gimbal_neut_flag,gimbal_neut_status_pub,0,0,kinect_params)
                    vel_msg.velocity.x = 0
                    vel_msg.velocity.z = 0
                    vel_msg.yaw_rate = 0.2
                
                # Test Code
                # vel_msg.velocity.x = 10       
                
                interceptor_gnc.local_vel_pub.publish(vel_msg)
                print("x,y,z,gimbal_yaw")
                print(x,y,z,yaw)
        toc = time.perf_counter()
        loop_time = toc - tic
        print("Loop Time: {}".format(loop_time))
        
        if first_loop:
            first_loop = 0
            with open(time_file_dir + str(timenum) +'_computation_time.csv','w') as f:
                writer = csv.writer(f)
                writer.writerow([loop_time,vel_msg.velocity.x])
        else:
            with open(time_file_dir + str(timenum) + '_computation_time.csv','a') as f:
                writer = csv.writer(f)
                writer.writerow([loop_time,vel_msg.velocity.x])
        rate.sleep()
    video.release()