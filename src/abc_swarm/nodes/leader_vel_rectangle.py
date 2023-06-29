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

def arctan2(y, x):
    a = np.arctan2(y, x)
    return a

class State:
    def __init__(self, d, a, hz=100, v=0.1, w=0.1):
        self.distance = d
        self.angle = a * 1.0 / 180 * np.pi
        self.hz = hz
        self.speed_v = v
        self.speed_w = w
        self.prev_pos = (0, 0)
        self.curr_pos = (0, 0)
        self.next_pos = (d, 0)
        self.theta = 0
        self.theta_list = []
        self.clock = 0
        self.tick = 0
        self.init_move_params()
    
    def init_move_params(self):
        self.state = 1
        dx = self.next_pos[0] - self.curr_pos[0]
        dy = self.next_pos[1] - self.curr_pos[1]
        distance = np.sqrt(dx * dx + dy * dy)   
        # assume distance > 10cm, a = 10cm/ss, 加速1s(恒定)，v=10cm/s， s=5cm
        self.clock = ((distance - self.speed_v + 0.05) / self.speed_v  + 2) * self.hz

    def init_rotate_params(self):
        self.state = 0
        dx = self.next_pos[0] - self.curr_pos[0]
        dy = self.next_pos[1] - self.curr_pos[1]
        angle = arctan2(dy, dx)
        if angle < 0:
            angle += np.pi * 2
        dt = angle - self.theta
        if dt < 0: dt += np.pi
        if dt > 2*np.pi : dt -= 2*np.pi
        print("dx= {}, dy = {}, angle = {}, theta = {}, dt = {}".format(dx, dy, angle, self.theta, dt))
        # assume angel > 0.1 * pi,  加速1s(恒定)，w=0.1， W = 0.05 * pi
        self.clock = ((dt  - self.speed_w)/ self.speed_w + 2) * self.hz

    def next_state(self):
        self.tick = 0
        if self.state == 1: # 直线结束
            alpha = arctan2(self.next_pos[1] - self.prev_pos[1], self.next_pos[0] - self.prev_pos[0])
            print(alpha, self.angle)
            pos_x = - self.distance * np.cos(alpha - self.angle) + self.next_pos[0]
            pos_y = - self.distance * np.sin(alpha - self.angle) + self.next_pos[1]
            self.prev_pos, self.next_pos = self.next_pos, (pos_x, pos_y)
            self.init_rotate_params()
        else: # 转弯结束
            self.init_move_params()
        print("at = ({}, {}), from = ({}, {}), to = ({}, {})".format(self.curr_pos[0], self.curr_pos[1], self.prev_pos[0], self.prev_pos[1], self.next_pos[0], self.next_pos[1]))

    def step(self):
        twist = Twist()
        self.tick += 1
        if self.tick >= self.clock + self.hz:
            self.next_state()
        if self.state == 1:
            if self.tick < self.hz: # 加速阶段
                twist.linear.x = self.speed_v * self.tick/ self.hz
            elif self.tick + self.hz < self.clock: # 平稳阶段
                twist.linear.x = self.speed_v
            elif self.tick < self.clock: # 减速阶段
                twist.linear.x = self.speed_v * (self.clock - self.tick) / self.hz
            else: 
                twist.linear.x = 0
        else:
            if self.tick < self.hz: # 加速阶段
                twist.angular.z = self.speed_w * self.tick/ self.hz
            elif self.tick + self.hz <= self.clock: # 平稳阶段
                twist.angular.z = self.speed_w
            elif self.tick < self.clock: # 减速阶段
                twist.angular.z = self.speed_w * (self.clock - self.tick) / self.hz
            else: 
                twist.angular.z = -self.speed_w * 0.001
        return twist


if __name__=="__main__":
    rospy.init_node("leader_draw_circle_by_vel")

    # 参数处理
    leader_name = rospy.get_param('~robot_name', 'bot1')
    dist = rospy.get_param('~expected_distiance', 0.5)
    angel = rospy.get_param('~expected_angel', 90)
    hz = rospy.get_param('~HZ', 20)
    v = rospy.get_param('~V', 0.1)
    w = rospy.get_param('~W', 0.1)
    state = State(dist, angel, hz, v, w)

    # 获取当前位置
    def odom_cb(msg):
        info = msg.pose.pose
        state.curr_pos = (info.position.x, info.position.y)
        # state.theta = np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) *2
        state.theta = tf.transformations.euler_from_quaternion([0, 0, info.orientation.z, info.orientation.w])[2]
    pub=rospy.Publisher(leader_name + "/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber(leader_name + "/odom", Odometry, odom_cb)
    print('111')
    rate = rospy.Rate(hz)
    rospy.sleep(3)
    print('222')
    
    while not rospy.is_shutdown():
        twist = state.step()
        pub.publish(twist)
        rate.sleep()
