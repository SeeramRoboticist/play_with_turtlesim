#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

x = 0.0
y = 0.0

def pose_cb(data):
    global x
    global y
    x = data.x
    y = data.y

def calculate_linear_velocity(x1, y1, x2, y2, dt):

    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance / dt

def calculate_angular_velocity(x1, y1, x2, y2, dt):

    angle1 = math.atan2(y1, x1)
    angle2 = math.atan2(y2, x2)
    angle_change = angle2 - angle1
    # Ensure the angle change stays within [-pi, pi] range
    if angle_change > math.pi:
        angle_change -= 2 * math.pi
    elif angle_change < -math.pi:
        angle_change += 2 * math.pi
    return angle_change / dt


rospy.init_node("move_turtle", anonymous=True)
rospy.Subscriber("/turtle/pose", Pose, pose_cb)


x_previous = x
y_previous = y
timestamp_previous = time.time()

while not rospy.is_shutdown():

    time.sleep(0.1)
    x_current, y_current = x, y

    time_stamp_current = time.time()
    dt = time_stamp_current - timestamp_previous

    print(x_previous, y_previous, x_current, y_current, dt)

    linear_velocit = calculate_linear_velocity(x_previous, y_previous, x_current, y_current, dt)
    angular_velocity = calculate_angular_velocity(x_previous, y_previous, x_current, y_current, dt)

    print(linear_velocit, angular_velocity)

    x_previous = x_current
    y_previous = y_current
    timestamp_previous = time_stamp_current
