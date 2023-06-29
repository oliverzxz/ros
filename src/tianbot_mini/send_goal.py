#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
import time
import numpy as np
import math
import tf
from geometry_msgs.msg import PoseStamped, Twist

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

rospy.init_node('send_goal')
# 创建一个tf.TransformListener对象
listener = tf.TransformListener()
# 循环读取tf数据
# rate = rospy.Rate(10.0)
orginal_translation = []
orginal_rotation = []
while not rospy.is_shutdown():
    try:
        # 读取tianbot_mini/base_link相对于map的坐标系转换关系
        (translation, rotation) = listener.lookupTransform('tianbot_mini/map', 'tianbot_mini/base_link', rospy.Time(0))
        # 输出位置和方向信息
        rospy.loginfo('tianbot_mini/base_link position: x=%f, y=%f, z=%f', translation[0], translation[1], translation[2])
        rospy.loginfo('tianbot_mini/base_link orientation: x=%f, y=%f, z=%f, w=%f', rotation[0], rotation[1], rotation[2], rotation[3])
        orginal_translation = translation
        orginal_rotation = rotation
        break
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
print(orginal_translation)
print(orginal_rotation)

# 从pos和ori中提取位置和朝向信息
x, y, z = orginal_translation[0], orginal_translation[1], orginal_translation[2]
qx, qy, qz, qw = orginal_rotation[0], orginal_rotation[1], orginal_rotation[2], orginal_rotation[3]

# 将朝向转换为欧拉角
yaw = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))

# 将位置和朝向信息转换为向量
pos_vec = np.array([x, y, z])
ori_vec = np.array([math.cos(yaw), math.sin(yaw), 0])


# 计算目标坐标
target_pos1 = pos_vec + 1 * ori_vec
target_pos2 = pos_vec + 2 * ori_vec
target_pos3 = pos_vec + 3 * ori_vec


# 输出结果
print('Current position:', pos_vec)
print('Current orientation:', ori_vec)
print('Target position:', target_pos3)



turtle_vel_pub = rospy.Publisher('tianbot_mini/move_base_simple/goal', PoseStamped, queue_size =10)
mypos=PoseStamped()
turtle_vel_pub.publish(mypos) #先发送一个空位置，试探一下，否则第一个包容易丢
time.sleep(1)

linear = 0.3
forward_time = 0.5
rospy.loginfo('左转')
turn_angular(0.0) # 测试
turn_angular(1.5)
time.sleep(2)
rospy.loginfo('直行')
move_forward(linear, 0.0, forward_time)
time.sleep(2)
rospy.loginfo('掉头')
turn_angular(-1.6)
time.sleep(1)
turn_angular(-1.5)
time.sleep(2)
rospy.loginfo('直行')
move_forward(linear, 0.0, 2*forward_time)
time.sleep(2)

mypos=PoseStamped()
mypos.header.frame_id='tianbot_mini/map' #设置自己的目标
mypos.header.stamp = rospy.Time.now()
mypos.pose.position.x=target_pos1[0]
mypos.pose.position.y=target_pos1[1]
mypos.pose.position.z=target_pos1[2]
mypos.pose.orientation.x=0
mypos.pose.orientation.y=0
mypos.pose.orientation.z=0
mypos.pose.orientation.w=1
turtle_vel_pub.publish(mypos) #发送自己设置的目标点
rospy.loginfo('第一个目标点发送完成')
time.sleep(5)
# 发送第二个目标点
mypos=PoseStamped()
mypos.header.frame_id='tianbot_mini/map' #设置自己的目标
mypos.header.stamp = rospy.Time.now()
mypos.pose.position.x=target_pos2[0]
mypos.pose.position.y=target_pos2[1]
mypos.pose.position.z=target_pos2[2]
mypos.pose.orientation.x=0
mypos.pose.orientation.y=0
mypos.pose.orientation.z=0
mypos.pose.orientation.w=1# 计算左侧和右侧的坐标
offset_vec_left = np.array([0.0, 0.3, 0.0])
rot_mat_left = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
offset_vec_right = np.array([0.0, -0.3, 0.0])
rot_mat_right = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])

target_pos_left = pos_vec + np.dot(rot_mat_left,ori_vec+offset_vec_left)
target_pos_right = pos_vec + np.dot(rot_mat_right,ori_vec+offset_vec_right)
time.sleep(5)
# 发送第三个目标点
mypos=PoseStamped()
mypos.header.frame_id='tianbot_mini/map' #设置自己的目标
mypos.header.stamp = rospy.Time.now()
mypos.pose.position.x=target_pos3[0]
mypos.pose.position.y=target_pos3[1]
mypos.pose.position.z=target_pos3[2]
mypos.pose.orientation.x=0
mypos.pose.orientation.y=0
mypos.pose.orientation.z=0
mypos.pose.orientation.w=1
turtle_vel_pub.publish(mypos) #发送自己设置的目标点
rospy.loginfo('第三个目标点发送完成')
time.sleep(5)
rospy.signal_shutdown("节点功能完成")