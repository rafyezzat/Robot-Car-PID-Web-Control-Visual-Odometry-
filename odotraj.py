import cv2
import numpy as np
import serial
import math
import time

# Serial connection to Arduino Mega
ser = serial.Serial("COM4", 115200, timeout=1)  # change COM port if needed
time.sleep(2)  # wait for connection

# Camera
cap = cv2.VideoCapture(0)  # change index if needed

# Parameters for optical flow
lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict(maxCorners=100,
                      qualityLevel=0.3,
                      minDistance=7,
                      blockSize=7)

# Initialize
ret, old_frame = cap.read()
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

traj_angle = 0

def send_command(cmd):
    print(f"Command: {cmd}")
    ser.write((cmd + "\n").encode())

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

    if p1 is not None and st is not None:
        good_new = p1[st == 1]
        good_old = p0[st == 1]

        # Calculate motion vectors
        dx, dy = 0, 0
        for (new, old) in zip(good_new, good_old):
            a, b = new.ravel()
            c, d = old.ravel()
            dx += (a - c)
            dy += (b - d)

        if len(good_new) > 0:
            dx /= len(good_new)
            dy /= len(good_new)

            angle = math.degrees(math.atan2(dy, dx))

            # Update trajectory angle
            traj_angle += angle * 0.01  # scaling factor

            # Decide command based on angle
            if abs(angle) < 10:
                send_command("F")  # Forward
            elif angle > 10:
                send_command("L")  # Left
            elif angle < -10:
                send_command("R")  # Right
            else:
                send_command("S")  # Stop

    # Show tracked features
    for pt in p0:
        x, y = pt.ravel()
        cv2.circle(frame, (int(x), int(y)), 3, (0, 255, 0), -1)

    cv2.imshow("Visual Odometry", frame)

    # Update previous frame and points
    old_gray = frame_gray.copy()
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
