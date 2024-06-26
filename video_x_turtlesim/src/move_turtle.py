#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import time


class moveTurtle:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        
        self.circle_pose_subscriber = rospy.Subscriber('/turtle/pose',Pose, self.circle_pose_cb)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def circle_pose_cb(self, circle_pose):

        self.circle_pose_x = round(circle_pose.x, 4)
        self.circle_pose_y = round(circle_pose.y, 4)
        self.move2goal(self.circle_pose_x, self.circle_pose_y)


    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=2.0):
        
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=10):
        
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)
        
    def move2goal(self, circle_pose_x, circle_pose_y):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        goal_pose.x = circle_pose_x
        goal_pose.y = circle_pose_y
        print(goal_pose)

        distance_tolerance = 1


        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        x = moveTurtle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass