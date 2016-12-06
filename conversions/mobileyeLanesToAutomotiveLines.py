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

def orientation_from_rpy(roll, pitch, yaw):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def confidence_from_mobileye_confidence(quality):
    if(quality == 0 or quality == 1):
        return 0.5
    else:
        return 1.0

def segment_from_lane(lane):
    segment = LineSegment()
    segment.pose.position.x = 0
    segment.pose.position.z = 0
    segment.pose.position.y = -lane.c0

    orientationQuaternion = orientation_from_rpy(0,0,-lane.c1)
    segment.pose.orientation.x = orientationQuaternion[0]
    segment.pose.orientation.y = orientationQuaternion[1]
    segment.pose.orientation.z = orientationQuaternion[2]
    segment.pose.orientation.w = orientationQuaternion[3]
    segment.c2 = -lane.c2
    segment.c3 = -lane.c3


    segment.length = lane.view_range
    segment.width = lane.width
    segment.type = lane.type
    segment.confidence = confidence_from_mobileye_confidence(lane.quality)
    return segment

def convert_to_lines(mobileye):
    lines = Lines()

    lines.header = mobileye.header
    lines.segments.append(segment_from_lane(mobileye.left_lane))
    lines.segments.append(segment_from_lane(mobileye.right_lane))

    return lines

def callback(c):
    global pub
    lines = convert_to_lines(c)
    pub.publish(lines)


##INIT
rospy.loginfo("Starting mobileyeLanesToAutomotiveLines")
rospy.init_node('mobileyeLanesToAutomotiveLines', anonymous=True)

pub = rospy.Publisher('/lines', Lines, queue_size=10)

rospy.Subscriber("/lanes", LaneInfo, callback)

rospy.spin()

rospy.loginfo("Finished mobileyeLanesToAutomotiveLines")
