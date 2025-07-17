#!/usr/bin/env python

import sys
import os
import time
import threading


# Cross-platform getch
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Load STServo SDK
sys.path.append("..")
from STservo_sdk import *
from STservo_sdk.sts import STS_ID

# === Configuration ===
BAUDRATE = 1000000
DEVICENAME = 'COM3'
STS_MOVING_SPEED = 2400
STS_MOVING_ACC = 50
ROTATION_TICKS = 4096  # One turn = 4096 ticks

POS_ZERO = 0
POS_PLUS_8 = 4096 * 8  # 32768 — one tick beyond 32767 would be invalid!
sts_goal_position = [POS_ZERO, 32767]  # Use max allowable position
index = 0  # Start by going to POS_ZERO

stop_polling = False
# User Configuration

print("Input current Motor ID:")
CURRENT_ID = int(input())
print("Current motor ID set to:", CURRENT_ID)

print("The ID which you want the motor to be changed to:")
NEW_ID = int(input())
print("Goal ID set to:", NEW_ID)

print("What mode do you want?: 0 (Position Control Mode), 1 (Wheel Mode)")
while True:
    try:
        DESIRED_Mode = int(input())
        if DESIRED_Mode == 0:
            print("Position Control Mode Selected")
            break
        elif DESIRED_Mode == 1:
            print("Wheel Mode selected")
            break
        else:
            print("Input must be 0 or 1")
    except ValueError:
        print("Invalid Input, must be 0 or 1")


# === Initialize Port & Packet Handler ===
portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)

if not portHandler.openPort():
    print("Failed to open port")
    getch()
    quit()
print("Port opened")

if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate")
    getch()
    quit()
print("Baudrate set")

# === Setup Servo: Position Mode & Limits ===
print("Configuring Servo")
packetHandler.unLockEprom(CURRENT_ID)
print("Unlocked EPROM")

def signed_to_unsigned_16bit(val):
        return val & 0xFFFF
def unsigned_to_signed_16bit(val):
    return val - 0x10000 if val > 0x7FFF else val




def poll_position(sts_id, handler):
    global stop_polling
    while not stop_polling:
        raw_pos, _, _ = handler.ReadPos(sts_id)
        signed_pos = unsigned_to_signed_16bit(raw_pos)
        print(f"📍 Current position: {signed_pos}")
        time.sleep(0.3)




if DESIRED_Mode == 0:
    packetHandler.write1ByteTxRx(CURRENT_ID, STS_MODE, 0)
    
    min_limit = 0
    max_limit =  17000

    packetHandler.write2ByteTxRx(CURRENT_ID, STS_MIN_ANGLE_LIMIT_L, min_limit & 0xFFFF)
    packetHandler.write2ByteTxRx(CURRENT_ID, STS_MAX_ANGLE_LIMIT_L, max_limit & 0xFFFF)

    print("Position COntrol Mode and angle limits set")
    min_val, _, _ = packetHandler.read2ByteTxRx(NEW_ID, STS_MIN_ANGLE_LIMIT_L)
    max_val, _, _ = packetHandler.read2ByteTxRx(NEW_ID, STS_MAX_ANGLE_LIMIT_L)
    print(f"✅ Limits set in EEPROM: {unsigned_to_signed_16bit(min_val)} to {unsigned_to_signed_16bit(max_val)}")


elif DESIRED_Mode ==1:
    packetHandler.write1ByteTxRx(CURRENT_ID, STS_MODE, 1)
    print("Wheel mode set")

result, error = packetHandler.write1ByteTxRx(CURRENT_ID, STS_ID, NEW_ID)
if result == COMM_SUCCESS and error == 0:
    print(f"Servo ID changed to {NEW_ID}")
else:
    print("Failed to change ID.")
packetHandler.LockEprom(NEW_ID)
print("EPROM Locked")

if DESIRED_Mode ==0:
    print("Will set the motor to the midway position, ready?")

      # Start polling thread
    stop_polling = False
    thread = threading.Thread(target=poll_position, args=(NEW_ID, packetHandler), daemon=True)
    thread.start()

    input()
    print(f"▶ Moving to signed position: {32766}")
    packetHandler.WriteSignedPosEx(NEW_ID, 32766, STS_MOVING_SPEED, STS_MOVING_ACC)

    input()
    print(f"▶ Moving to signed position: {0}")
    packetHandler.WriteSignedPosEx(NEW_ID, 0, STS_MOVING_SPEED, STS_MOVING_ACC)

    time.sleep(0.5)
    input()

    # Stop polling thread
    stop_polling = True
    thread.join()

if DESIRED_Mode ==1:
    while True:
        key = getch()
        if key == chr(0x1b):  # ESC key
            break
        elif key.lower() == 'q':
            print(NEW_ID, "Clockwise")
            sts_comm_result, sts_error = packetHandler.WriteSpec(NEW_ID, STS_MOVING_SPEED, STS_MOVING_ACC)
        elif key.lower() == 'a':
            print(NEW_ID, "Counter-clockwise")
            sts_comm_result, sts_error = packetHandler.WriteSpec(NEW_ID, -STS_MOVING_SPEED, STS_MOVING_ACC)
        elif key == chr(32):
            print(NEW_ID, "Stop All")
            sts_comm_result, sts_error = packetHandler.WriteSpec(NEW_ID, 0, STS_MOVING_ACC)
