#!/usr/bin/env python

import sys
import os
import pygame
import time
import numpy as np
import matplotlib.pyplot as plt


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
motor_IDS = [1,2,3,4,5]
starting_angles=[0,67.7,-148,48,0]
BAUDRATE = 1000000             # Default baudrate
DEVICENAME = 'COM3'            # Change this to match port (Linux: '/dev/ttyUSB0')

TICKS_PER_TURN = 4096

STS_MOVING_SPEED = 1200  # Pattern of speeds
STS_MOVING_ACC = 50
        #Motors = [1, 2, 3, 4, 5]
target_position = []
target_angle = []
maxLimits = []
minLimits = []
raw_position = []
prev_pos = {}
turn_count = {}
abs_positions = {}
abs_angle_positions ={}
max_angle=[]
output_position = []

# For logging accuracy over time
time_log = []
target_angle_log = {sid: [] for sid in motor_IDS}
actual_angle_log = {sid: [] for sid in motor_IDS}
error_angle_log = {sid: [] for sid in motor_IDS}
velocity_log = {sid: [] for sid in motor_IDS}
acceleration_log = {sid: [] for sid in motor_IDS}


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

def unsigned_to_signed_16bit(val):
    return val - 0x10000 if val > 0x7FFF else val




def compute_accuracy():
    errors = []
    print("\n--- Accuracy Report ---")
    for i, sid in enumerate(motor_IDS):
        target_pos_ticks = output_position[i]  # Target position in encoder ticks
        actual_pos_ticks = abs_positions[sid]  # Measured absolute position in ticks

        error_ticks = actual_pos_ticks - target_pos_ticks
        error_deg = error_ticks * (18 / TICKS_PER_TURN)  # Convert to degrees

        print(f"Motor {sid}:")
        print(f"  Target Pos (ticks): {target_pos_ticks}")
        print(f"  Actual Pos (ticks): {actual_pos_ticks}")
        print(f"  Error: {error_ticks} ticks | {error_deg:.2f} degrees")

        errors.append(error_deg)

    # Summary stats
    mean_error = np.mean(errors)
    std_dev_error = np.std(errors)
    max_error = np.max(np.abs(errors))

    print("\nSummary:")
    print(f"  Mean Error: {mean_error:.2f}°")
    print(f"  Std Dev: {std_dev_error:.2f}°")
    print(f"  Max Error: {max_error:.2f}°")
    print("-----------------------\n")

def plot_accuracy():
    print("Generating accuracy plots...")
    for sid in motor_IDS:
        plt.figure(figsize=(10, 5))
        plt.title(f"Motor {sid} - Target vs Actual Angle Over Time")
        plt.plot(time_log, target_angle_log[sid], label='Target Angle (°)')
        plt.plot(time_log, actual_angle_log[sid], label='Actual Angle (°)')
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (°)")
        plt.legend()
        plt.grid(True)
        plt.show()

        plt.figure(figsize=(10, 4))
        plt.title(f"Motor {sid} - Error Over Time")
        plt.plot(time_log, error_angle_log[sid], label='Error (Target - Actual)', linestyle='--')
        plt.xlabel("Time (s)")
        plt.ylabel("Error (°)")
        plt.legend()
        plt.grid(True)
        plt.show()
def plot_joint_trajectory():
    print("Plotting joint trajectory smoothness...")

    for sid in motor_IDS:
        # Velocity Plot
        t_vel = time_log[1:]                          # Velocity starts at 2nd time point
        v_log = velocity_log[sid][1:]                 # Align lengths
        plt.figure(figsize=(10, 4))
        plt.title(f"Motor {sid} - Joint Velocity Over Time")
        plt.plot(t_vel, v_log, label='Velocity (°/s)')
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (°/s)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

        # Acceleration Plot
        t_acc = time_log[2:]                          # Acceleration starts at 3rd time point
        a_log = acceleration_log[sid][2:]             # Align lengths
        plt.figure(figsize=(10, 4))
        plt.title(f"Motor {sid} - Joint Acceleration Over Time")
        plt.plot(t_acc, a_log, label='Acceleration (°/s²)', color='orange')
        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration (°/s²)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()


def usefulinfo():
    raw_position =[]
    for sid in motor_IDS:
        
        raw_pos, _, _ = packetHandler.ReadPos(sid)
        raw_position.append(unsigned_to_signed_16bit(raw_pos))

        
        
    print("Absolute Position:", abs_positions)
    print("Angles (absolute)", abs_angle_positions)
    print("Raw Position:", raw_position)
    print("Target Position:", target_position)
    print("Output Position:", output_position)

for sid in motor_IDS:
    target_position.append(0)
    output_position.append(0)
    target_angle.append(starting_angles[sid-1])
    raw_pos, _, _ = packetHandler.ReadPos(sid)
    raw_position.append(unsigned_to_signed_16bit(raw_pos))
    prev_pos[sid] = unsigned_to_signed_16bit(raw_pos)
    turn_count[sid] = 0
    abs_positions[sid] = 0
    abs_angle_positions[sid] = starting_angles[sid -1]
    max_val, _, _ = packetHandler.read2ByteTxRx(sid, STS_MAX_ANGLE_LIMIT_L)
    min_val, _, _ = packetHandler.read2ByteTxRx(sid, STS_MIN_ANGLE_LIMIT_L)
    maxangle = (unsigned_to_signed_16bit(max_val) * (18/TICKS_PER_TURN)) + starting_angles[sid - 1] # 18 because 20:1 ratio, 360/20 =18
    max_angle.append(maxangle)
    maxLimits.append(unsigned_to_signed_16bit(max_val))
    minLimits.append(unsigned_to_signed_16bit(min_val))

    
print("Max Limits:", maxLimits)
print("Max Limits:", minLimits)
print("Starting Angles:", starting_angles)
print("Max Angles:", max_angle)
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

    # Sending the array of angles to the motor
    for sid in motor_IDS:
        output_position[sid-1] = int((target_angle[sid-1]-starting_angles[sid-1]) * (TICKS_PER_TURN / 18))
        packetHandler.WriteSignedPosEx(sid, output_position[sid - 1], STS_MOVING_SPEED, STS_MOVING_ACC)

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
        abs_angle_positions[sid] = starting_angles[sid-1] + (abs_positions[sid] * (18/TICKS_PER_TURN))
    
    if now - last_update_time > update_interval:
        time_log.append(now)

        for i, sid in enumerate(motor_IDS):
            tgt_angle = target_angle[i]
            act_angle = abs_angle_positions[sid]
            err_angle = tgt_angle - act_angle

            target_angle_log[sid].append(tgt_angle)
            actual_angle_log[sid].append(act_angle)
            error_angle_log[sid].append(err_angle)
            
        for sid in motor_IDS:
            angle_list = actual_angle_log[sid]

            # Velocity
            if len(time_log) >= 2:
                dt = time_log[-1] - time_log[-2]
                dtheta = angle_list[-1] - angle_list[-2]
                velocity = dtheta / dt
            else:
                velocity = 0.0
            velocity_log[sid].append(velocity)

            # Acceleration
            if len(time_log) >= 3:  # We need at least 3 timestamps to compute 2 velocities
                dt_vel = time_log[-2] - time_log[-3]
                dv = velocity_log[sid][-1] - velocity_log[sid][-2]
                acceleration = dv / dt_vel
            else:
                acceleration = 0.0
            acceleration_log[sid].append(acceleration)



    
    if joystick.get_button(8):
        usefulinfo()
        print("Compute_accuracy")
        compute_accuracy()
        time.sleep(0.5)

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
        # Servo 1
        if joystick.get_button(5):
            
            target_position[0] = min(maxLimits[0], target_position[0] + STS_MOVING_SPEED)
            target_angle[0] = starting_angles[0] + (target_position[0] * (18/TICKS_PER_TURN))
            print(motor_IDS[0], "Clockwise", target_angle)
            print(target_angle)
        elif joystick.get_button(4):
            target_position[0] = max(0, target_position[0] - STS_MOVING_SPEED)
            target_angle[0] = starting_angles[0] + (target_position[0] * (18/TICKS_PER_TURN))
            print(motor_IDS[0], "Counter-clockwise", target_angle)
            print(target_angle)

        # Servo 2
        if axis_valy > 0.1:
            target_position[1] = min(maxLimits[1], target_position[1] + STS_MOVING_SPEED)
            target_angle[1] = starting_angles[1] + (target_position[1] * (18/TICKS_PER_TURN))
            print(motor_IDS[1], "Clockwise", target_angle)
            print(target_angle)
        elif axis_valy < -0.1:
            target_position[1] = max(0, target_position[1] - STS_MOVING_SPEED)
            target_angle[1] = starting_angles[1] + (target_position[1] * (18/TICKS_PER_TURN))
            print(motor_IDS[1], "Counter-clockwise", target_angle)
            print(target_angle)

        # Servo 3
        if axis_valx > 0.1:
            target_position[2] = min(maxLimits[2], target_position[2] + STS_MOVING_SPEED)
            target_angle[2] = starting_angles[2] + (target_position[2] * (18/TICKS_PER_TURN))
            print(motor_IDS[2], "Clockwise", target_angle)
        elif axis_valx < -0.1:
            target_position[2] = max(0, target_position[2] - STS_MOVING_SPEED)
            target_angle[2] = starting_angles[2] + (target_position[2] * (18/TICKS_PER_TURN))
            print(motor_IDS[2], "Counter-clockwise",target_angle)

        # Servo 4
        if joystick.get_button(0):
            target_position[3] = min(maxLimits[3], target_position[3] + STS_MOVING_SPEED)
            target_angle[3] = starting_angles[3] + (target_position[3] * (18/TICKS_PER_TURN))
            print(motor_IDS[3], "Clockwise", target_angle)
        elif joystick.get_button(3):
            target_position[3] = max(0, target_position[3] - STS_MOVING_SPEED)
            target_angle[3] = starting_angles[3] + (target_position[3] * (18/TICKS_PER_TURN))
            print(motor_IDS[3], "Counter-clockwise", target_angle)
        
        # Servo 5
        if joystick.get_button(1):
            target_position[4] = min(maxLimits[4], target_position[4] + STS_MOVING_SPEED)
            target_angle[4] = starting_angles[4] + (target_position[4] * (18/TICKS_PER_TURN))
            print(motor_IDS[4], "Clockwise", target_angle)
        elif joystick.get_button(2):
            target_position[4] = max(0, target_position[4] - STS_MOVING_SPEED)
            target_angle[4] = starting_angles[4] + (target_position[4] * (18/TICKS_PER_TURN))
            print(motor_IDS[4], "Counter-clockwise", target_angle)

        last_update_time = now

#     elif joystick.get_button(8):
#         print(motor_IDS[0], "Stop All")
#         for sid in motor_IDS:
#             sts_comm_result, sts_error = packetHandler.WriteSpec(sid, 0, STS_MOVING_ACC)




# # # Stop the motor before exiting
# # for sid in motor_IDS:
# #     packetHandler.WriteSignedPosEx(sid, 0, STS_MOVING_SPEED, STS_MOVING_ACC)


# plot_accuracy()
# plot_joint_trajectory()


portHandler.closePort()
print("Port closed. Motors stopped. Program exited.")