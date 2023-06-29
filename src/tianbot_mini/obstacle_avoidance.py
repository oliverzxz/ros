#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
from math import pi

state = 1


def index_func(a, a_min, a_inc):
    return int(((a * pi) / 180 - a_min) / a_inc)


def main():
    rospy.init_node('obstacle_avoidance', anonymous=True)
    rospy.Subscriber('tianbot_mini/scan', LaserScan, laser_callback, queue_size=1)
    rospy.Subscriber('tianbot_mini/map', OccupancyGrid, costmap_callback)
    rospy.Subscriber('tianbot_mini/command_completed', String, command_callback)
    rospy.spin()


def command_callback(msg):
    if msg.data == 'command_completed':
        rospy.loginfo('command execution has been completed!')
    else:
        rospy.loginfp('error')


def get_obstacle(start_angle, end_angle, data, detect_distance):
    # 这个函数的功能是，判断指定视角内是否存在指定判断距离的障碍物，返回值为bool类型
    # print(data)
    fliter_list = []
    for i in range(index_func(start_angle, data.angle_min, data.angle_increment),
                   index_func(end_angle, data.angle_min, data.angle_increment)):
        if data.ranges[i] >= 0.1:  # 大于0.1是因为当前雷达可探测的距离大于0.1米
            fliter_list.append(data.ranges[i])
    if len(fliter_list) >= 1:
        if min(fliter_list) <= detect_distance:
            return 1
        else:
            return 0
    else:
        return 0


def move_forward(lin, ang, move_time):
    pub = rospy.Publisher('tianbot_mini/cmd_vel', Twist, queue_size=1)
    move_cmd = Twist()
    move_cmd.linear.x = lin
    move_cmd.angular.z = ang
    duration = rospy.Duration.from_sec(move_time)  # 运行时间为5秒
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time) < duration:
        pub.publish(move_cmd)
        rospy.sleep(duration)
    # 运行完之后发布停止命令
    stop_cmd = Twist()
    stop_cmd.linear.x = 0
    stop_cmd.angular.z = 0
    pub.publish(stop_cmd)


def turn_angular(ang):
    pub = rospy.Publisher('tianbot_mini/cmd_vel', Twist, queue_size=1)
    turn_cmd = Twist()
    turn_cmd.linear.x = 0.0
    turn_cmd.angular.z = ang
    pub.publish(turn_cmd)
    rospy.sleep(1)
    turn_cmd.angular.z = 0
    pub.publish(turn_cmd)


def move_judge(data):
    detect_distance = 0.3  # 障碍物探测距离参数
    pi_state = [0, 0, 0, 0, 0, 0]  # 将雷达数据划分为8个扇区
    pi_state[0] = get_obstacle(0, 60, data, detect_distance)
    pi_state[1] = get_obstacle(60, 120, data, detect_distance)
    pi_state[2] = get_obstacle(120, 180, data, detect_distance)
    pi_state[3] = get_obstacle(-60, 0, data, detect_distance)
    pi_state[4] = get_obstacle(-120, -60, data, detect_distance)
    pi_state[5] = get_obstacle(-180, -120, data, detect_distance)
    print(pi_state)
    linear = 0.25
    angular = 0.3
    forward_time = 0.2
    if pi_state == [0, 0, 0, 0, 0, 0]:
        # 此时直行
        move_forward(linear, 0.0, forward_time)
        print('直行')
    elif pi_state in [[1, 0, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0], [1, 0, 0, 1, 0, 0]]:
        # 此时右转X度
        turn_angular(-angular)
        print('右转')
    elif pi_state in [[0, 1, 0, 0, 0, 0]]:
        # 此时再向前走X厘米
        move_forward(linear, 0.0, forward_time)
        print('直行')
    elif pi_state in [[0, 1, 1, 0, 0, 0], [0, 0, 1, 0, 0, 0]]:
        # 此时左转X度
        turn_angular(angular)
        print('左转')
    elif pi_state in [[1, 1, 1, 1, 0, 0]]:
        # 此时再向前走X厘米
        move_forward(linear, 0.0, forward_time)
        print('直行')
    elif pi_state in [[0, 0, 0, 1, 0, 0], [0, 0, 0, 1, 0, 0]]:
        # 此时左转X度
        turn_angular(angular)
        print('左转')


def laser_callback(data):
    """
    激光雷达数据的回调函数
    """
    move_judge(data)


def costmap_callback(data):
    pub_goal = rospy.Publisher('tianbot_mini/move_base_simple/goal', PoseStamped, queue_size=2)
    position_x = data.info.origin.position.x
    position_y = data.info.origin.position.y
    orientation_z = data.info.origin.orientation.z
    orientation_w = data.info.origin.orientation.w
    goal_point = PoseStamped()
    goal_point.pose.position.x = position_x + 3.0
    goal_point.pose.position.y = position_y
    goal_point.header.stamp = rospy.Time.now()
    goal_point.pose.orientation.z = orientation_z
    goal_point.pose.orientation.w = orientation_w
    goal_point.header.frame_id = 'map'
    rospy.sleep(2)
    # 将目标点坐标发布
    pub_goal.publish(goal_point)
    print('初始坐标：')
    print(position_x, position_y)
    # 根据初始坐标和机器人方向角计算目标位置坐标
    print('终点坐标：')
    print(goal_point.pose.position.x, goal_point.pose.position.y)


if __name__ == '__main__':
    main()
