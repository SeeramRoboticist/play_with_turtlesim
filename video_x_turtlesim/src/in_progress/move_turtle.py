#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


class moveTurtle:

    def __init__(self) -> None:

        self.move_turtle_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/turtle1/pose", Pose, self.turtle_pose_cb)
        rospy.Subscriber("/turtle/pose", Pose, self.pose_cb)
        self.rate = rospy.Rate(10)

        #Turtle_pose
        self.turtle_x = None
        self.turtle_y = None
        self.turtle_theta = None

        #Goal_pose
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None

    def turtle_pose_cb(self, turtle_pose):

        self.turtle_x = turtle_pose.x
        self.turtle_y = turtle_pose.y
        self.turtle_theta = turtle_pose.theta

    def pose_cb(self, goal_pose):

        self.goal_x = goal_pose.x
        self.goal_y = goal_pose.y
        self.goal_theta = goal_pose.theta

        twist_msg = Twist()

        # kp_liner = 0.4
        distance = math.sqrt(((self.goal_x - self.turtle_x) ** 2) + ((self.goal_y - self.goal_y) ** 2))
        # vx = distance 

        # kp_angular = 4
        desired_angle_goal = math.atan2(self.goal_y - self.turtle_y, self.goal_x - self.turtle_y)
        # v_theta = (desired_angle_goal - self.turtle_theta) * kp_angular

        distance_tolerance = 0.1
        angle_tolerance = 0.01

        angle_error = desired_angle_goal - self.turtle_theta
        kp = 10
        if abs(angle_error) > angle_tolerance:
            twist_msg.angular.z = kp * angle_error
        else:
            if (distance) >= distance_tolerance:
                twist_msg.linear.x = kp*distance
            else:
                twist_msg.linear.x = 0.0
                quit()

        # twist_msg.linear.x = vx
        # twist_msg.angular.z = v_theta

        self.move_turtle_pub.publish(twist_msg)
        self.rate.sleep()



if __name__ == "__main__":
    rospy.init_node("move_turtle", anonymous=True)
    move_turtle = moveTurtle()
    rospy.spin()