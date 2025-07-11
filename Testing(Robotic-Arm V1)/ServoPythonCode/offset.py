#!/usr/bin/env python

import sys

# Load STServo SDK
sys.path.append("..")
from STservo_sdk import *
import time

# === CONFIG ===
STS_ID = 1
BAUDRATE = 1000000
DEVICENAME = 'COM5'
OFFSET_START = 16383  # midpoint (our logical zero)
STS_MOVING_SPEED = 2400
STS_MOVING_ACC = 50

# Setup Communication
portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)

if not portHandler.openPort():
    print("❌ Failed to open port")
    exit(1)
portHandler.setBaudRate(BAUDRATE)

# === SET HARDWARE OFFSET TO 16383 ===
offset_low = OFFSET_START & 0xFF
offset_high = (OFFSET_START >> 8) & 0xFF


packetHandler.write1ByteTxRx(STS_ID, STS_OFS_L, offset_low)
packetHandler.write1ByteTxRx(STS_ID, STS_OFS_H, offset_high)

print(f"✅ Offset set to {OFFSET_START}")

# === SET MODE TO POSITION CONTROL (MODE 0) ===
packetHandler.write1ByteTxRx( STS_ID, STS_MODE, 0)

# === MOVEMENT SEQUENCE USING OFFSET-BASED LOGIC ===

input("Press Enter to move to logical position 0 (== 16383 real)...")
print(f"▶ Moving to logical 0 (== 16383 real)")
packetHandler.WritePosExOff(STS_ID, 0, STS_MOVING_SPEED, STS_MOVING_ACC)

input("Press Enter to move to +4 turns (== 32766 real)...")
print(f"▶ Moving to +4 turns (== 32766 real)")
packetHandler.WritePosExOff(STS_ID, 32766 - OFFSET_START, STS_MOVING_SPEED, STS_MOVING_ACC)

input("Press Enter to move to -4 turns (== 0 real)...")
print(f"▶ Moving to -4 turns (== 0 real)")
packetHandler.WritePosExOff(STS_ID, -OFFSET_START, STS_MOVING_SPEED, STS_MOVING_ACC)
