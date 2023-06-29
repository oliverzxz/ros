#! /usr/bin/env python
#triangle
#patrol_points = [[0., 0.], [0., 0.4], [0.34641, 0.2]]

from pickle import TRUE
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rospy.timer import Timer
import math
import numpy as np
import tf
from math import radians

pi = math.pi
t = 2.0
#patrol_points = [[0., 0.], [0.5, 0.]]
patrol_points = [[0., 0.], [0., 0.5 * t], [0.433 * t, 0.25 * t]]
# patrol_points = [[0., 0. ], [0.5 * t, 0.], [0.5 * t, 0.5 * t], [0., 0.5 * t]]

'''
patrol_points = [[0., 0.], 
                                    [-t*math.cos(18 / 180 * pi), -t*math.sin(18 / 180 * pi) + t], 
                                    [t*math.cos(54 / 180 * pi), t*math.sin(54 / 180 * pi) + t],
                                    [t*math.cos(18 /180 * pi), -t*math.sin(18 /180*pi) + t],
                                    [-t*math.cos(54 / 180 * pi), t*math.sin(54 / 180 * pi) + t]]
#                                  [t*math.cos(54 ./ 180 * pi), t*math.sin(54 ./ 180 * pi) + t], 
#                                  [t*math.cos(18./180 * pi), - t*math.sin(18./180*pi) + t]]
'''

angle = 0.0
dist = 0.0
max_speed = 0.2

twist=Twist()

#initial points
x0 = 0
y0 = 0

#goal points
x_ = 0
y_ = 0

init = False
i = 0

def subCallback(odom):
    global x0
    global y0
    global init
    global x_
    global y_
    global i
    if not init:
        x0 = odom.pose.pose.position.x
        y0 = odom.pose.pose.position.y
        init = True
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    euler = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
    global angle

    angle = np.arctan2(y_ - y, x_ - x) - euler[2]
    if angle > pi:
        angle -= 2 * pi
    if angle <= -pi:
        angle += 2 * pi

    global dist
    dist = math.sqrt((x - x_)**2 + (y - y_) ** 2)
    rospy.loginfo("current position : (%f, %f), distance to the goal: %f, angle to adjust: %f",  x, y, dist, angle)
    if dist < 0.02:
        print("hit goal point: (%f, %f)", x_, y_)
        i = i + 1 if i + 1 < length else 0
        init = False
        [x_, y_]= patrol_points[i]
    
def speed(d):
    # intercept = math.sqrt(x_ ** 2 + y_ ** 2) * 2 / max_speed
    intercept = math.sqrt((x_  - x0)** 2 + (y_ - y0)** 2) * 2 / max_speed
    return  math.sqrt((d * 2  * max_speed)/ intercept)


def timeCallback(event):
    pub=rospy.Publisher("bot2/cmd_vel", Twist, queue_size=1)
    # if abs(angle) > pi / 60:
    #     twist.linear.x = 0
    #     if angle > 0:
    #         twist.angular.z = angular_speed
    #     else:
    #         twist.angular.z = -angular_speed
    # elif dist >= 0.02:
    #     twist.linear.x = speed(dist)
    #     twist.angular.z = 0
    # else:
    #     twist.linear.x = 0
    #     twist.angular.z = 0       
    # if abs(angle) > pi / 12:
    #     twist.linear.x = 0
    #     twist.angular.z = angle
    if dist >= 0.05:
        twist.linear.x = speed(dist)
        twist.angular.z = angle
    elif dist > 0.02:
        twist.linear.x = speed(dist)
        twist.angular.z = 0
    else:
        twist.linear.x = 0
        twist.angular.z = 0     
    pub.publish(twist)
    rospy.sleep(1)

if __name__=="__main__":
    length = len(patrol_points)
    rospy.init_node("velandanglecontrol")

    sub = rospy.Subscriber("bot2/odom", Odometry , subCallback, queue_size=10)

    rospy.Timer(rospy.Duration(1), timeCallback, oneshot=False)

    rospy.spin()

    
    pass
