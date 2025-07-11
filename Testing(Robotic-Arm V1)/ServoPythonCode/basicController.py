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
motor_IDS = [1,2,3,4]
BAUDRATE = 1000000             # Default baudrate
DEVICENAME = 'COM3'            # Change this to match port (Linux: '/dev/ttyUSB0')

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

# Initialize Pygame and Joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick detected. Plug in your SNES controller.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Detected: {joystick.get_name()}")

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
    pygame.event.pump()
    axis_valy = joystick.get_axis(0)
    axis_valx = joystick.get_axis(1)
    if joystick.get_button(9):  # ESC key
        break
    elif joystick.get_button(5):
        print(motor_IDS[0], "Clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[0], STS_MOVING_SPEED, STS_MOVING_ACC)
    elif joystick.get_button(4):
        print(motor_IDS[0], "Counter-clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[0], -STS_MOVING_SPEED, STS_MOVING_ACC)
    elif axis_valy > 0.1:
        print(motor_IDS[1], "Clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[1], STS_MOVING_SPEED, STS_MOVING_ACC)
    elif axis_valy < -0.1:
        print(motor_IDS[1], "Counter-clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[1], -STS_MOVING_SPEED, STS_MOVING_ACC)
    elif axis_valx > 0.1:
        print(motor_IDS[2], "Clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[2], STS_MOVING_SPEED, STS_MOVING_ACC)
    elif axis_valx < -0.1:
        print(motor_IDS[2], "Counter-clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[2], -STS_MOVING_SPEED, STS_MOVING_ACC)
    elif joystick.get_button(0):
        print(motor_IDS[3], "Clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[3], STS_MOVING_SPEED, STS_MOVING_ACC)
    elif joystick.get_button(3):
        print(motor_IDS[3], "Counter-clockwise")
        sts_comm_result, sts_error = packetHandler.WriteSpec(motor_IDS[3], -STS_MOVING_SPEED, STS_MOVING_ACC)
    elif joystick.get_button(8):
        print(motor_IDS[0], "Stop All")
        for sid in motor_IDS:
            sts_comm_result, sts_error = packetHandler.WriteSpec(sid, 0, STS_MOVING_ACC)


# Stop the motor before exiting
for sid in motor_IDS:
    packetHandler.WriteSpec(sid, 0, STS_MOVING_ACC )

portHandler.closePort()
print("Port closed. Motor stopped. Program exited.")