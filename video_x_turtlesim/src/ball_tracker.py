#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from turtlesim.msg import Pose
from cv_bridge import CvBridge
bridge = CvBridge()

class ballTracker:

    def __init__(self) -> None:

        self.pose_pub = rospy.Publisher("/turtle/pose", Pose, queue_size = 1)
        self.animation_sub = rospy.Subscriber("/image/ball_animation", Image, self.animation_cb, queue_size = 1)
        self.frame = None

    def animation_cb(self, animation_data):

        self.frame = bridge.imgmsg_to_cv2(animation_data, 'bgr8')

        gray_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        blur_frame = cv2.GaussianBlur(gray_frame, (17,17), 8)

        circles = cv2.HoughCircles(blur_frame, cv2.HOUGH_GRADIENT, 1.2, 180, 
                                   param1=100, param2=30, minRadius=10, maxRadius=400)
        
        circles = np.uint16(np.round(circles))
        circles = circles[0][0]
        print(circles)

        """To view the Circle in ouput uncomment below lines"""

        # cv2.circle(self.frame, (circles[0], circles[1]), 1, (0,100,100), 3)
        # cv2.circle(self.frame, (circles[0], circles[1]), circles[2], (255,255,100), 3)
        # cv2.imshow("frame", self.frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()
        #     self.animation_sub.unregister()


    def pose_publisher(self):

        while not rospy.is_shutdown():

            cv2.imshow("frame", self.frame)
            if cv2.waitKeqy(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("ball_tracker", anonymous=True)
    track = ballTracker()
    rospy.spin()