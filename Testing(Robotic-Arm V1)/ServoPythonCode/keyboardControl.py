#!/usr/bin/env python

import sys
import os
import pygame

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from STservo_sdk import *  # Uses STServo SDK library

# Settings
# STS_IDS = [4]               # Servo IDs
motor_IDS = [1,2,3,4,5,6,7]
BAUDRATE = 1000000             # Default baudrate
DEVICENAME = 'COM5'            # Change this to match port (Linux: '/dev/ttyUSB0')

STS_MOVING_SPEED = 2400  # Pattern of speeds
STS_MOVING_ACC = 50

# Setup

portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)

if not portHandler.openPort():
    print("Failed to open port.")
    sys.exit()

if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate.")
    sys.exit()

# Enable wheel mode
for sid in motor_IDS:
    result, error = packetHandler.WheelMode(sid)
    if result != COMM_SUCCESS:
        print(f"ID {sid}: {packetHandler.getTxRxResult(result)}")
    elif error != 0:
        print(f"ID {sid}: {packetHandler.getRxPacketError(error)}")
    else:
        print(f"ID {sid}: Wheel mode enabled.")

# Main
while True:
    key = getch()
    if key == chr(0x1b):  # ESC key
        break
    elif key.lower() == 'q':
        print(motor_IDS[0], "Clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[0], STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 'a':
        print(motor_IDS[0], "Counter-clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[0], -STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 'w':
        print(motor_IDS[1], "Clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[1], STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 's':
        print(motor_IDS[1], "Counter-clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[1], -STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 'e':
        print(motor_IDS[2], "Clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[2], STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 'd':
        print(motor_IDS[2], "Counter-clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[2], -STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 'r':
        print(motor_IDS[3], "Clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[3], STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 'f':
        print(motor_IDS[3], "Counter-clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[3], -STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 't':
        print(motor_IDS[4], "Clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[4], STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 'g':
        print(motor_IDS[4], "Counter-clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[4], -STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 'y':
        print(motor_IDS[5], "Clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[5], STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 'h':
        print(motor_IDS[5], "Counter-clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[5], -STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 'u':
        print(motor_IDS[6], "Clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[6], STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key.lower() == 'j':
        print(motor_IDS[6], "Counter-clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[6], -STS_MOVING_SPEED, STS_MOVING_ACC)
    elif key == chr(32):
        print(motor_IDS[0], "Stop All")
        for sid in motor_IDS:
            sts_comm_result, sts_error = packetHandler.WriteSpec(sid, 0, STS_MOVING_ACC)
    else:
        print(f"⏹ Unknown key '{key}' — no action taken.")


# Stop the motor before exiting
for sid in motor_IDS:
    packetHandler.WriteSpec(sid, 0, STS_MOVING_ACC )

portHandler.closePort()
print("Port closed. Motor stopped. Program exited.")
