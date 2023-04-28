import cv2
import numpy as np
import serial
import RPi.GPIO as GPIO
from pymavlink import mavutil

# Constants
TARGET_PATTERN = 'path/to/target_pattern.png'
WEBCAM_INDEX = 0
SERIAL_PORT = '/dev/serial0'  # Change to the appropriate serial port
BAUD_RATE = 57600  # Change to the appropriate baud rate

# Configure GPIO 15 (RX) and 16 (TX) for serial communication
GPIO.setmode(GPIO.BOARD)
GPIO.setup(15, GPIO.IN)
GPIO.setup(16, GPIO.OUT)

# Establish MAVLink connection
mav = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)

# Initialize USB webcam
cap = cv2.VideoCapture(WEBCAM_INDEX)

# Check if the webcam is connected
if not cap.isOpened():
    print("Error: Webcam is not connected or not detected.")
    exit(1)

# Load target pattern
target = cv2.imread(TARGET_PATTERN, cv2.IMREAD_GRAYSCALE)


# Function to identify the target in the frame
def find_target(frame):
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.matchTemplate(gray_frame, target, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    return max_loc if max_val > 0.5 else None


# Function to compute MAVLink commands based on the target position
def control_drone(target_position):
    x, y = target_position
    x_center, y_center = width // 2, height // 2
    x_diff, y_diff = x_center - x, y_center - y

    if abs(x_diff) > 10:
        if x_diff > 0:
            mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                                      1, 0, 0, 0, 0, 0, 0)
        else:
            mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                                      -1, 0, 0, 0, 0, 0, 0)

    if abs(y_diff) > 10:
        if y_diff > 0:
            mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                                      0, 1, 0, 0, 0, 0, 0, 0)
        else:
            mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                                      0, -1, 0, 0, 0, 0, 0, 0)


# Main loop
while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Unable to receive a video signal from the webcam.")
        break

    target_position = find_target(frame)

    if target_position:
        control_drone(target_position)
        print("Target detected at position: {}".format(target_position))
    else:
        print("Target not detected")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


"C:\Users\harry\Downloads\target.jpg"
TARGET_PATTERN = '/home/pi/target_pattern.png'
pscp "C:/Users/harry/Downloads/target_pattern.png" pi@raspberrypi.local:/home/pi/
'/home/pi/target_pattern.png'
