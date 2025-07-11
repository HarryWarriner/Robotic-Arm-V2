#!/usr/bin/env python

import sys
import os
import pygame
import time

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
motor_IDS = [1]
BAUDRATE = 1000000             # Default baudrate
DEVICENAME = 'COM5'            # Change this to match port (Linux: '/dev/ttyUSB0')

TICKS_PER_TURN = 4096

STS_MOVING_SPEED = 2400  # Pattern of speeds
STS_MOVING_ACC = 50
        #Motors = [1, 2, 3, 4, 5]
target_position = []
maxLimits = []
minLimits = []
raw_position = []
prev_pos = {}
turn_count = {}
abs_positions = {}


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

def unsigned_to_signed_16bit(val):
    return val - 0x10000 if val > 0x7FFF else val




def usefulinfo():
    raw_position =[]
    for sid in motor_IDS:
        
        raw_pos, _, _ = packetHandler.ReadPos(sid)
        raw_position.append(unsigned_to_signed_16bit(raw_pos))

        
        
    print("Absolute Position:", abs_positions)
    print("Raw Position:", raw_position)
    print("Target Position:", target_position)

for sid in motor_IDS:
    target_position.append(0)
    raw_pos, _, _ = packetHandler.ReadPos(sid)
    raw_position.append(unsigned_to_signed_16bit(raw_pos))
    prev_pos[sid] = unsigned_to_signed_16bit(raw_pos)
    turn_count[sid] = 0
    abs_positions[sid] = 0
    max_val, _, _ = packetHandler.read2ByteTxRx(sid, STS_MAX_ANGLE_LIMIT_L)
    min_val, _, _ = packetHandler.read2ByteTxRx(sid, STS_MIN_ANGLE_LIMIT_L)
    maxLimits.append(unsigned_to_signed_16bit(max_val))
    minLimits.append(unsigned_to_signed_16bit(min_val))
print("Max Limits:", maxLimits)
print("Max Limits:", minLimits)
usefulinfo()





last_update_time = time.time()
update_interval = 0.1

# Main
running = True
while running:
    
    pygame.event.pump()
    now =time.time()
    
    axis_valy = joystick.get_axis(0)
    axis_valx = joystick.get_axis(1)

    for sid in motor_IDS:
        packetHandler.WriteSignedPosEx(sid, target_position[sid - 1], STS_MOVING_SPEED, STS_MOVING_ACC)

    for sid in motor_IDS:
        raw_pos, _, _ = packetHandler.ReadPos(sid)
        current_pos = unsigned_to_signed_16bit(raw_pos)
        delta = current_pos - prev_pos[sid]

        if delta > TICKS_PER_TURN / 2:
            turn_count[sid] -= 1
        elif delta < -TICKS_PER_TURN / 2:
            turn_count[sid] += 1



        abs_positions[sid] = current_pos + turn_count[sid] * TICKS_PER_TURN
        prev_pos[sid] = current_pos
    
    if joystick.get_button(8):  # ESC button
        usefulinfo()
        time.sleep(0.5)

    if joystick.get_button(9):  # ESC button
        running = False
        break

    if now - last_update_time > update_interval:
        # Servo 1
        if joystick.get_button(5):
            
            target_position[0] = min(maxLimits[0], target_position[0] + STS_MOVING_SPEED)
            print(motor_IDS[0], "Clockwise", target_position)
        elif joystick.get_button(4):
            target_position[0] = max(0, target_position[0] - STS_MOVING_SPEED)
            print(motor_IDS[0], "Counter-clockwise", target_position)

        # Servo 2
        if axis_valy > 0.1:
            target_position[1] = min(maxLimits[1], target_position[1] + STS_MOVING_SPEED)
            print(motor_IDS[1], "Clockwise", target_position)
        elif axis_valy < -0.1:
            target_position[1] = max(0, target_position[1] - STS_MOVING_SPEED)
            print(motor_IDS[1], "Counter-clockwise", target_position)

        # Servo 3
        if axis_valx > 0.1:
            target_position[2] = min(maxLimits[2], target_position[2] + STS_MOVING_SPEED)
            print(motor_IDS[2], "Clockwise", target_position)
        elif axis_valx < -0.1:
            target_position[2] = max(0, target_position[2] - STS_MOVING_SPEED)
            print(motor_IDS[2], "Counter-clockwise", target_position)

        # Servo 4
        if joystick.get_button(0):
            target_position[3] = min(maxLimits[3], target_position[3] + STS_MOVING_SPEED)
            print(motor_IDS[3], "Clockwise", target_position)
        elif joystick.get_button(3):
            target_position[3] = max(0, target_position[3] - STS_MOVING_SPEED)
            print(motor_IDS[3], "Counter-clockwise", target_position)

        last_update_time = now

#     elif joystick.get_button(8):
#         print(motor_IDS[0], "Stop All")
#         for sid in motor_IDS:
#             sts_comm_result, sts_error = packetHandler.WriteSpec(sid, 0, STS_MOVING_ACC)




# # # Stop the motor before exiting
# # for sid in motor_IDS:
# #     packetHandler.WriteSignedPosEx(sid, 0, STS_MOVING_SPEED, STS_MOVING_ACC)

portHandler.closePort()
print("Port closed. Motors stopped. Program exited.")