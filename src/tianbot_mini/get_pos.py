#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
import time
import tf
from geometry_msgs.msg import PoseStamped


rospy.init_node('get_pos')
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
rospy.signal_shutdown("节点功能完成，节点关闭!")