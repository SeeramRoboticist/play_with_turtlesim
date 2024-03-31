#!/usr/bin/env python3
import cv2
import numpy as np
import math
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()

#Ininitializing ROS and Waiting for Subscribers
rospy.init_node("video_publisher", anonymous=True)

video_publisher = rospy.Publisher('/image/ball_animation', Image, queue_size=1)

if video_publisher.get_num_connections() == 0:
    rospy.logwarn("No Subscribers yet for the topic")
while video_publisher.get_num_connections() == 0:
    pass
rospy.loginfo("Stream Started")

# Function to generate Lemniscate path
def lemniscate_path(a, t):
    x = a * math.cos(t) / (1 + math.sin(t) ** 2)
    y = a * math.cos(t) * math.sin(t) / (1 + math.sin(t) ** 2)
    return int(x), int(y)

# Parameters
a = 150  # Adjust as needed
frame_width = 500
frame_height = 500
center_x = frame_width // 2
center_y = frame_height // 2


# Generate frames
num_frames = 200
stream_video = True
while stream_video:

    for i in range(50 , num_frames + 50):
        t = 2 * math.pi * i / num_frames
        x, y = lemniscate_path(a, t)

        # Create blank frame
        frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        cv2.line(frame, (0, center_y), (frame_width, center_y), (255, 255, 255), 1)
        cv2.line(frame, (center_x, 0), (center_x, frame_height), (255, 255, 255), 1)
        
        # Draw green ball (circle)
        ball_radius = 20
        cv2.circle(frame, (center_x + x, center_y + y), ball_radius, (0, 255, 0), -1)
        print(center_x + x, center_y + y)
        
        # Show frame and Publish it to rostopic
        cv2.imshow('Lemniscate Motion', frame)
        cv = bridge.cv2_to_imgmsg(frame, 'bgr8')
        video_publisher.publish(cv)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            stream_video = False
            break   