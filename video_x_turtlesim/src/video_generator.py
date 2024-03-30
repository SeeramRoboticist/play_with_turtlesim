import cv2
import numpy as np
import math

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
run_for = True
for i in range(50 , num_frames + 50):
    t = 2 * math.pi * i / num_frames
    print(a,t)
    x, y = lemniscate_path(a, t)

    # Create blank frame
    frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
    cv2.line(frame, (0, center_y), (frame_width, center_y), (255, 255, 255), 1)
    cv2.line(frame, (center_x, 0), (center_x, frame_height), (255, 255, 255), 1)
    
    # Draw green ball (circle)
    ball_radius = 15
    cv2.circle(frame, (center_x + x, center_y + y), ball_radius, (0, 255, 0), -1)
    # print(center_x + x, center_y + y)
    
    # Show frame
    cv2.imshow('Lemniscate Motion', frame)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        # cv2.destroyAllWindows()
        run_for = False
        break

cv2.destroyAllWindows()