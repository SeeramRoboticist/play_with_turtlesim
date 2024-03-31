#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class LemniscateMotionController:
    def __init__(self):
        rospy.init_node('lemniscate_motion_controller', anonymous=True)
        self.pose_sub = rospy.Subscriber('/turtle/pose', Pose, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.turtle_pose = Pose()
    
    def pose_callback(self, data):
        self.turtle_pose = data
    
    def run(self):
        while not rospy.is_shutdown():
            cmd_vel = Twist()
            # Lemniscate motion calculation
            x = self.turtle_pose.x
            y = self.turtle_pose.y
            
            # Calculate linear velocity using lemniscate equation
            v = 0.1  # Constant linear velocity
            a = 1  # Scale factor for the lemniscate
            b = 0.5
            omega = 0.5  # Angular velocity
            cmd_vel.linear.x = v * (a ** 2 * b ** 2) / ((x ** 2 + y ** 2) ** 2 + 2 * (a ** 2 + b ** 2) * (x ** 2 - y ** 2) + (a ** 2 - b ** 2) ** 2)
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            
            # Calculate angular velocity
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = omega
            
            # Publish the command velocity
            self.cmd_vel_pub.publish(cmd_vel)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = LemniscateMotionController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
