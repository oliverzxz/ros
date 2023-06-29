#! /usr/bin/env python
# coding:utf-8 
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rospy.timer import Timer
import math
import numpy as np
import tf
import time


if __name__=="__main__":
    rospy.init_node("leader_draw_circle_by_vel")
    leader_name = rospy.get_param('~robot_name', 'bot1')
    radis = rospy.get_param('~expected_radis', 0.5)

    pub=rospy.Publisher(leader_name + "/cmd_vel", Twist, queue_size=1)
    twist=Twist()
    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown():  # 2pi *1 / 0.1 = 2pi / w
        twist.linear.x = 0.3
        twist.angular.z = 0.3
        pub.publish(twist)
        rate.sleep()