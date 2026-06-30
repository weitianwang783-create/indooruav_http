#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion, PoseWithCovariance, TwistWithCovariance

rospy.init_node('mock_odometry', anonymous=True)

pub_odom = rospy.Publisher('/Odometry_global', Odometry, queue_size=10)
pub_px = rospy.Publisher('/Odometry_px', Point, queue_size=10)

rate = rospy.Rate(5)
t = 0.0

while not rospy.is_shutdown():
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "map"

    odom.pose.pose.position.x = 10.0 + math.sin(t * 0.5) * 2.0
    odom.pose.pose.position.y = 20.0 + math.cos(t * 0.5) * 2.0
    odom.pose.pose.position.z = 5.0

    yaw = t * 0.3
    odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
    odom.pose.pose.orientation.w = math.cos(yaw * 0.5)

    odom.twist.twist.linear.x = 0.5
    odom.twist.twist.linear.z = 0.1

    pub_odom.publish(odom)

    px = Point()
    px.x = 150.0 + math.sin(t * 0.5) * 20.0
    px.y = 200.0 + math.cos(t * 0.5) * 20.0
    px.z = 0.0
    pub_px.publish(px)

    t += 0.2
    rate.sleep()
