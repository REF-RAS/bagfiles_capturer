#!/usr/bin/env python3

# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

# -- ROS modules
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import opencv_tools as opencv_tools
import cv_bridge
# from rosbags.image import message_to_cvimage

# -- converts the image_cv as a ROS Image Msg object 
def create_image_msg(image_cv):
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.height = image_cv.shape[0] # number of rows
    msg.width = image_cv.shape[1]
    if len(image_cv.shape) == 3:
        msg.step = image_cv.shape[2] * msg.width  # number of bytes in a row
    else:
        msg.step = msg.width
    msg.encoding = 'bgr8'
    msg.is_bigendian = False
    msg.data = opencv_tools.array_to_bytes(image_cv)
    return msg

def depthimage_to_imagecv(depth_image:Image):
    bridge = cv_bridge.CvBridge()
    depth_image = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
    return depth_image

def rgbimage_to_imagecv(rbg_image:Image):
    bridge = cv_bridge.CvBridge()
    image_cv = bridge.imgmsg_to_cv2(rbg_image, desired_encoding='gbr8')
    # image_cv = message_to_cvimage(image_cv, 'bgr8')
    return image_cv    

# extract the parts from the incoming command string for a service
# the command string is a comma-separated list [command, param1, param2, ...]
def extract_command_string(command_string, detect_number=True):
    parts = command_string.split(',')
    if detect_number:
        for i, param in enumerate(list(parts)):
            if i == 0: continue
            try:
                x = int(param)
                parts[i] = x
            except:
                try:
                    x = float(param)
                    parts[i] = x
                except:
                    pass
    return parts