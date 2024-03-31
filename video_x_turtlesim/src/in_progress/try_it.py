#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


class moveTurtle:

    def __init__(self) -> None:

        self.move_turtle_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
        if self.move_turtle_pub.get_num_connections() == 0:
            rospy.logwarn("Please run the turtlesim node")
        while self.move_turtle_pub.get_num_connections() == 0:
            pass
        rospy.loginfo("Lets Go")
        self.dummy = rospy.Subscriber("/turtle/pose", Pose, self.pose_cb, queue_size=1)
        self.dummy = rospy.Subscriber("/turtle1/pose", Pose, self.turtle_pose_cb, queue_size=1)
        time.sleep(1)

        self.twist = Twist()

        #Goal_pose
        # self.x_pose = 0.0
        # self.y_pose = 0.0
        self.goal_theta = None


    def pose_cb(self, goal_pose):

        self.x_pose = goal_pose.x
        self.y_pose = goal_pose.y

    def turtle_pose_cb(self, turtle_pose):
        self.turtle_pose_x = turtle_pose.x
        self.turtle_pose_y = turtle_pose.y

    def calculate_linear_velocity(self, x1, y1, x2, y2, dt):

        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance / dt

    def calculate_angular_velocity(self, x1, y1, x2, y2, dt):

        angle1 = math.atan2(y1, x1)
        angle2 = math.atan2(y2, x2)
        angle_change = angle2 - angle1
        # Ensure the angle change stays within [-pi, pi] range
        if angle_change > math.pi:
            angle_change -= 2 * math.pi
        elif angle_change < -math.pi:
            angle_change += 2 * math.pi
        return angle_change / dt
    
    def move(self, speed):

        x0 = self.turtle_pose_x
        y0 = self.turtle_pose_y


    
    def velocity_computer(self):

        x_previous = self.x_pose
        y_previous = self.y_pose
        print(x_previous, y_previous)

        timestamp_previous = time.time()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            time.sleep(0.1)
            x_current, y_current = self.x_pose, self.y_pose
            # print(f"after{x_current, y_current}")
            # break

            time_stamp_current = time.time()
            dt = time_stamp_current - timestamp_previous

            linear_velocit = self.calculate_linear_velocity(x_previous, y_previous, x_current, y_current, dt)
            angular_velocity = self.calculate_angular_velocity(x_previous, y_previous, x_current, y_current, dt)

            print(linear_velocit, angular_velocity)
            self.twist.linear.x = abs(linear_velocit*0.5)
            self.twist.angular.z = abs(angular_velocity*2)
            self.move_turtle_pub.publish(self.twist)
            rate.sleep() 


            x_previous = x_current
            y_previous = y_current
            timestamp_previous = time_stamp_current




if __name__ == "__main__":
    rospy.init_node("move_turtle", anonymous=True)
    move_turtle = moveTurtle()
    move_turtle.velocity_computer()