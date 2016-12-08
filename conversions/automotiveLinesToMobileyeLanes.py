#!/usr/bin/python

import rospy

import ctypes
import math
import struct
import sys

import tf

from geometry_msgs.msg import Pose
from mobileye_msgs.msg import Lane, LaneInfo
from automotive_sensor_msgs.msg import Lines, LineSegment

def rpy_from_quaternion(quaternion):
    #Assuming x,y,z,w for quaternion
    rpy = tf.transformations.euler_from_quaternion(quaternion)
    return rpy

def confidence_to_mobileye_quality(confidence):
    if(confidence <= 0.25):
        return Lane.LOW_QUALITY_1
    elif(confidence <= 0.5):
        return Lane.LOW_QUALITY_2
    elif(confidence <= 0.75):
        return Lane.HIGH_QUALITY_1
    else:
        return Lane.HIGH_QUALITY_2

def convert_to_lane(line):
    lane = Lane()
    lane.type = line.type
    lane.model_degree = Lane.THIRD_DEGREE_MODEL
    
    lane.quality = confidence_to_mobileye_quality(line.confidence)
    lane.c0 = -line.pose.position.y
    rpy = rpy_from_quaternion([line.pose.orientation.x, line.pose.orientation.y, line.pose.orientation.z, line.pose.orientation.w])
    lane.c1 = -rpy[2] #yaw
    lane.c2 = -line.c2
    lane.c3 = -line.c3
    lane.width = line.width
    lane.view_range = line.length

    return lane

def convert_to_lanes(lines):
    lanes = LaneInfo()
    lanes.header = lines.header
    nLines = len(lines.segments)
    #Rightmost two lines
    lanes.left_lane = convert_to_lane(lines.segments[1])
    lanes.right_lane = convert_to_lane(lines.segments[0])
    lanes.yaw = 0
    lanes.pitch = 0
    lanes.c1 = -1*(lanes.left_lane.c1 + lanes.right_lane.c1)/2.0
    lanes.c2 = -1*(lanes.left_lane.c2 + lanes.right_lane.c2)/2.0
    return lanes

def callback(c):
    global pub
    lines = convert_to_lanes(c)
    pub.publish(lines)


##INIT
rospy.loginfo("Starting AutomotiveLanesToMobileyeLanes")
rospy.init_node('AutomotiveLanesToMobileyeLanes', anonymous=True)

pub = rospy.Publisher('/lanes', LaneInfo, queue_size=10)

rospy.Subscriber("/lines", Lines, callback)

rospy.spin()

rospy.loginfo("Finished AutomotiveLanesToMobileyeLanes")
