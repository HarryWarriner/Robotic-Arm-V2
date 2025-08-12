#!/usr/bin/env python

'''
Requires:
STservo_sdk
'''


import sys
import time
import os
import threading


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

# Default settings
STS_ID = 1
MOTOR_IDS = [1]
BAUDRATE = 1000000
DEVICENAME  = 'COM6'
 # e.g. Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
SPEED = 1000
ACC = 100

# Threading settings
print_lock = threading.Lock()
monitor_stop = threading.Event()
comm_lock = threading.Lock()


# Initalise PortHandler instance and PacketHandler instance
portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)
    
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable position mode
for sid in MOTOR_IDS:
    result, error = packetHandler.write1ByteTxRx(sid, STS_MODE, 3)
    if result != COMM_SUCCESS:
        print(f"ID {sid}: {packetHandler.getTxRxResult(result)}")
    elif error != 0:
        print(f"ID {sid}: {packetHandler.getRxPacketError(error)}")
    else:
        print(f"ID {sid}: Position mode enabled.")


input()
# Turn of the limits by setting them both to 0:
# (Still can only tunr +- 8 turns from midpoint (start point))

sts_comm_result, sts_error= packetHandler.Write2Byte(STS_ID, 0x09, 0) # Minimum angle
print(packetHandler.getTxRxResult(sts_comm_result))
sts_comm_result, sts_error= packetHandler.Write2Byte(STS_ID, 0x0b, 0) # Maxium angle
print(packetHandler.getTxRxResult(sts_comm_result))

# Read the limits to verify:

result, sts_comm_result, sts_error= packetHandler.Read2Byte(STS_ID, 0x09) # Minimum angle
print(packetHandler.getTxRxResult(sts_comm_result), "Minimum angle:", result)

result, sts_comm_result, sts_error= packetHandler.Read2Byte(STS_ID, 0x0b) # Maxium angle
print(packetHandler.getTxRxResult(sts_comm_result), "Maximum angle:", result)

pos, sts_comm_result, sts_error = packetHandler.ReadPos(STS_ID)
if sts_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(sts_comm_result))
else:
    global_position = pos

def monitor_loop(poll_hz=10):
    interval = 1.0 / poll_hz
    while not monitor_stop.wait(interval):
        try:
            showPosition(STS_ID)
        except Exception as e:
            with print_lock:
                print("Monitor error:", e)
            time.sleep(interval)

def showPosition(ID):
    # with comm_lock:
    #     result, sts_comm_result, sts_error= packetHandler.Read2Byte(ID, 67) # Unknown
    # with print_lock:
    #     print("Code Absolute Position:", result)  

    with comm_lock:
        # Read STServo present position, speed, acceleration, current
        pos, speed, acc, current, sts_comm_result, sts_error = packetHandler.ReadPosSpeedAccCurrent(ID)
        result, sts_comm_result, sts_error= packetHandler.Read2Byte(ID, 67)

    with print_lock:
        if sts_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(sts_comm_result))
        else:
            print(
                "[ID:%03d] Pos:%d Speed:%d Acc:%d (~%d steps/s^2) Current:%d global_position:%d byte67:%d"
                % (ID, pos, speed, acc, acc * 100, current, global_position, result)
            )
        if sts_error != 0:
            print(packetHandler.getRxPacketError(sts_error))
    

def moveBySteps(ID, steps, speed, acc):
    global global_position
    # print("Moving by steps:", steps)
    global_position += steps
    if steps<0:
        steps = -steps + 0x8000  # Convert to negative steps
    
    with comm_lock:
        sts_comm_result, sts_error = packetHandler.WritePosEx(ID, steps, speed, acc)

    if sts_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(sts_comm_result))
    elif sts_error != 0:
        print("%s" % packetHandler.getRxPacketError(sts_error))
      
    while True:
      with comm_lock:
        result, sts_comm_result, sts_error= packetHandler.ReadByte(ID, 0x42) # Is it moving?
        
      if sts_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(sts_comm_result))
      elif sts_error != 0:
        print("%s" % packetHandler.getRxPacketError(sts_error))
      else:
        # print("[ID:%03d] Moving status: %d" % (ID, result))
        if result==0:
        #   print("Servo stopped moving")
          break
        else:
          time.sleep(0.1)


# def moveBySteps(ID, steps, speed, acc):
#     global global_position
#     print("Moving by steps:", steps)
#     with comm_lock:
#         cur_pos, comm, err= packetHandler.Read2Byte(ID, 67) # Unknown
#     if comm != COMM_SUCCESS:
#         print(packetHandler.getTxRxResult(comm)); return
#     if err != 0:
#         print(packetHandler.getRxPacketError(err)); return
    
#     if steps<0:
#         steps = -steps + 0x8000  # Convert to negative steps

#     target = (global_position + steps) & 0xFFFF
#     global_position += steps

#     with comm_lock:
#         sts_comm_result, sts_error = packetHandler.WritePosEx(ID, target, speed, acc)
#     if sts_comm_result != COMM_SUCCESS:
#         print(packetHandler.getTxRxResult(sts_comm_result)); return
#     if sts_error != 0:
#         print(packetHandler.getRxPacketError(sts_error)); return

#     # poll moving flag
#     while True:
#         with comm_lock:
#             moving, comm2, err2 = packetHandler.ReadByte(ID, STS_MOVING)  # 0x42
#         if comm2 != COMM_SUCCESS:
#             print(packetHandler.getTxRxResult(comm2)); break
#         if err2 != 0:
#             print(packetHandler.getRxPacketError(err2)); break
#         if moving == 0:
#             break
#         time.sleep(0.05)


# Start Threading:

monitor_thread = threading.Thread(target=monitor_loop, kwargs={"poll_hz": 10}, daemon=True)
monitor_thread.start()        
        

while 1:
    # showPosition(STS_ID)
    # print("Press i-info, p-pos, -zero, a- -100, b +100, q-quit")
    ch = getch()
    if ch== chr(0x1b) or ch == 'q':
        break
    elif ch == 'i':
      # Read STServo present position
      for index in range(0, 70):
        result, sts_comm_result, sts_error= packetHandler.ReadByte(STS_ID, index)
        if sts_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(sts_comm_result))
        else:
            print("[ID:%03d] Position:%d =  %d" % (STS_ID, index, result))
        if sts_error != 0:
            print(packetHandler.getRxPacketError(sts_error))
    elif ch == 'p':
        showPosition(STS_ID)
    elif ch == '-':
      result, sts_comm_result, sts_error= packetHandler.Read2Byte(STS_ID, 67) # Unknown
      print("Current:", result)  
      moveBySteps(STS_ID, -result, SPEED, ACC) # Move to zero position
    elif ch == 'a' or ch=='b':
      if ch=='a':
          moveBySteps(STS_ID, -1024, SPEED, ACC)
      else:
          moveBySteps(STS_ID, +1024, SPEED, ACC)
# Stop thread, close port
monitor_stop.set()
monitor_thread.join(timeout=1.0)
portHandler.closePort()