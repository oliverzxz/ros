#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
from math import radians

t = 1.0
# patrol_squre = [[[0., 0.], 0], [[0.5 * t, 0.], 90], [[0.5 * t, 0.5 * t], 180], [[0., 0.5 * t], -90]]

class DrawAShape():
    def __init__(self):
        # initiliaze
        shape = "triangle"
        to_just = 0
        if shape == "squre":
            to_just = 90
        elif shape == "triangle":
            to_just = 120

        dist = t

        rospy.init_node('draw shape', anonymous=False)

        # What to do you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        self.cmd_vel = rospy.Publisher('bot1/cmd_vel', Twist, queue_size=20)
     
    # 5 HZ
        self.rate = 5
        angle_speed = 30
        linear_speed = 0.20
        r = rospy.Rate(self.rate);

    # create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.

        # let's go forward at 0.2 m/s
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
    # by default angular.z is 0 so setting this isn't required

        #let's turn at 45 deg/s
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(angle_speed); #45 deg/s in radians/s

        #two keep drawing squares.  Go forward for 2 seconds (10 x 5 HZ) then turn for 2 second
        # i = 0
        # angle_last = 0
        # last_x, last_y = 0., 0.
        for k in range(50):
            self.cmd_vel.publish(Twist())
            r.sleep()

        while not rospy.is_shutdown():
            # go forward 0.5 m (2 seconds * 0.2 m / seconds)
            # [goal, angle] = way_points[i]
            # dist = ((goal[0] - last_x) ** 2 + (goal[1] - last_y) ** 2) ** 0.5
            # to_just = angle - angle_last
            # if to_just < -180:
            #     to_just += 360
            # if dist <= 0.01 and to_just == 0:
            #     rospy.loginfo("Hit goal %d, [%f, %f]", i, goal[0], goal[1])
            #     i = i+1 if i < len(way_points)-1 else 0
            #     angle_last = angle
            #     [last_x, last_y] = goal
            #     continue
            rospy.loginfo("Turning")
            if to_just % (angle_speed / self.rate) != 0:
                rospy.loginfo("Can't hit the goal")
                break
            for x in range(to_just * self.rate / angle_speed):
                self.cmd_vel.publish(turn_cmd)
                r.sleep()    
            rospy.loginfo("Going Straight")
            for x in range(int(dist * self.rate / linear_speed)):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
    
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop Drawing Squares")

        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        # rospy.signal_shutdown()
 
if __name__ == '__main__':
    try:
        DrawAShape()
    except:
        rospy.loginfo("node terminated.")