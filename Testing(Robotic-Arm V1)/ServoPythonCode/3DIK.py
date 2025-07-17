#!/usr/bin/env python

import sys
import os
import pygame
import time
import numpy as np

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
from STservo_sdk import *

# Settings
motor_IDS = [1,2,3,4,5]
starting_angles=[0,67.7,-148,48,0]
BAUDRATE = 1000000             
DEVICENAME = 'COM4'            # Change this to match port (Linux: '/dev/ttyUSB0')

TICKS_PER_TURN = 4096
STS_MOVING_SPEED = 1200
STS_MOVING_ACC = 50

# Inital 3d target:
x = 10
y = 0
z= 15
phi = -np.pi / 2  # orientation

output_position = []
target_angle = []

# Link lengths in cm
l1 = 22.85
l2 = 22.85
l3 = 24.25  




# Setup

portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)

if not portHandler.openPort():
    print("Failed to open port.")
    sys.exit()

if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate.")
    sys.exit()

# Initalise Pygame and Joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick detected. Plug in your SNES controller.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Detected: {joystick.get_name()}")

# Enable position mode
for sid in motor_IDS:
    result, error = packetHandler.write1ByteTxRx(sid, STS_MODE, 0)
    if result != COMM_SUCCESS:
        print(f"ID {sid}: {packetHandler.getTxRxResult(result)}")
    elif error != 0:
        print(f"ID {sid}: {packetHandler.getRxPacketError(error)}")
    else:
        print(f"ID {sid}: Position mode enabled.")

# setup arrays:
for sid in motor_IDS:
    target_angle.append(starting_angles[sid-1])
    output_position.append(0)

# Send all motors to 0
print("Sending To Zero")
for sid in motor_IDS:
    packetHandler.WriteSignedPosEx(sid, 0, STS_MOVING_SPEED, STS_MOVING_ACC)
time.sleep(1)
# Wait until all motors have stopped moving
all_stopped = False
while not all_stopped:
    all_stopped = True
    for sid in motor_IDS:
        moving, _, _ = packetHandler.ReadMoving(sid)
        if moving != 0:
            all_stopped = False
            break
    time.sleep(0.05)  
print("At Zero")


def analytical_ik_3dof_with_base(x, y, z, phi):
    """
    Full 3D IK:
    - x, y: horizontal coordinates (base plane)
    - z: height
    - phi: desired orientation in the XZ plane

    Returns [θ0, θ1, θ2, θ3] in degrees or None if unreachable.
    """
    theta0 = np.arctan2(y, x)  # Base rotation

    r = np.sqrt(x**2 + y**2)   # Horizontal distance from base
    wx = r - l3 * np.cos(phi)
    wz = z - l3 * np.sin(phi)

    # Distance squared to wrist
    r_sq = wx**2 + wz**2
    cos_theta2 = (r_sq - l1**2 - l2**2) / (2 * l1 * l2)
    
    if abs(cos_theta2) > 1:
        return None

    theta2 = np.arccos(cos_theta2)
    k1 = l1 + l2 * np.cos(theta2)
    k2 = l2 * np.sin(theta2)
    theta1 = np.arctan2(wz, wx) - np.arctan2(k2, k1)
    theta3 = phi - theta1 - theta2

    # Return angles in degrees
    return np.degrees([theta0, theta1, theta2, theta3])

def update_position(x, y, z, phi):
    angles = analytical_ik_3dof_with_base(x, y, z, phi)
    if angles is None:
            print("Unreachable.")
            return
    
    theta0, theta1, theta2, theta3 = angles
    print(f"θ0: {theta0:.1f}, θ1: {theta1:.1f}, θ2: {theta2:.1f}, θ3: {theta3:.1f}")
    print("Output", output_position)
    target_angle[0] = int(theta0)
    target_angle[1] = int(theta1)
    target_angle[2] = int(-theta2)
    target_angle[3] = int(450-theta3)
    print(target_angle)



last_update_time = time.time()
update_interval = 0.1
# Main
running = True
while running:
    
    pygame.event.pump()
    now =time.time()
    
    axis_valy = joystick.get_axis(0)
    axis_valx = joystick.get_axis(1)

    # Sending the array of angles to the motor
    for sid in motor_IDS:
        output_position[sid-1] = int((target_angle[sid-1]-starting_angles[sid-1]) * (TICKS_PER_TURN / 18))
        packetHandler.WriteSignedPosEx(sid, output_position[sid - 1], STS_MOVING_SPEED, STS_MOVING_ACC)


    if joystick.get_button(9):  # ESC button
        # Send all motors to 0
        print("Sending To Zero")
        for sid in motor_IDS:
            packetHandler.WriteSignedPosEx(sid, 0, STS_MOVING_SPEED, STS_MOVING_ACC)
        time.sleep(1)
        # Wait until all motors have stopped moving
        all_stopped = False
        while not all_stopped:
            all_stopped = True
            for sid in motor_IDS:
                moving, _, _ = packetHandler.ReadMoving(sid)
                if moving != 0:
                    all_stopped = False
                    break
            time.sleep(0.05)  
        print("At Zero")
        running = False
        break
    if now - last_update_time > update_interval:
        # x axis
        if axis_valx > 0.1:
            y -= 1
            update_position(x,y,z,phi)
            print(target_angle , x,y,z)
        elif axis_valx < -0.1:
            y += 1
            update_position(x,y,z,phi)
            print(target_angle , x,y,z)

        # y axis
        if axis_valy > 0.1:
            x += 1
            update_position(x,y,z,phi)
            print(target_angle , x,y,z)
        elif axis_valy < -0.1:
            x -= 1
            update_position(x,y,z,phi)
            print(target_angle , x,y,z)

        if joystick.get_button(1):
            z += 1
            update_position(x,y,z,phi)
            print(target_angle , x,y,z)
        elif joystick.get_button(2):
            z -= 1
            update_position(x,y,z,phi)
            print(target_angle , x,y,z)
        last_update_time = now


portHandler.closePort()
print("Port closed. Motors stopped. Program exited.")