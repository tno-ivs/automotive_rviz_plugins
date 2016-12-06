#!/usr/bin/env python

import rospy
from math import sin, cos
from automotive_sensor_msgs.msg import Lines, LineSegment

from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

rospy.init_node("publish_lines_msg")
pub = rospy.Publisher("/lines", Lines, queue_size=20)

timeStart = rospy.Time.now()
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    msg = Lines()
    msg.header.frame_id = "/map"
    msg.header.stamp = rospy.Time.now()

    # Change c1
    segment = LineSegment()
    segment.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 2 * rospy.Time.now().to_sec()))
    segment.pose.position.x = 20
    segment.pose.position.y = 20
    segment.pose.position.z = 0
    segment.c2 = 0.01
    segment.c3 = 0.0
    segment.length = 50
    segment.width = 3.7
    segment.type = 0
    segment.confidence = 1
    msg.segments.append(segment)

    # Change c2
#    segment = LineSegment()
#    segment.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 2))
#    segment.pose.position.x = 0
#    segment.pose.position.y = 0
#    segment.pose.position.z = 0
#    segment.c2 = sin(0.01*(rospy.Time.now()-timeStart).to_sec())
#    segment.c3 = 0.0
#    segment.length = 20
#    segment.width = 3.7
#    segment.type = 1
#    segment.confidence = 1
#    msg.segments.append(segment)

    # Change c2
#    segment = LineSegment()
#    segment.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 2))
#    segment.pose.position.x = -20
#    segment.pose.position.y = -20
#    segment.pose.position.z = 0
#    segment.c2 = 0.0
#    segment.c3 = sin(0.001*(rospy.Time.now()-timeStart).to_sec())
#    segment.length = 20
#    segment.width = 3.7
#    segment.type = 2
#    segment.confidence = 1
#    msg.segments.append(segment)

    pub.publish(msg)

    rate.sleep()
