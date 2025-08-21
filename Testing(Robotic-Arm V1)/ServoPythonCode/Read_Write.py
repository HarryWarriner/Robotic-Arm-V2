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
DEVICENAME  = 'COM3'
 # e.g. Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
SPEED = 1000
ACC = 100

# Ticks
TICKS_PER_TURN = 4096
HALF_TURN = TICKS_PER_TURN // 2

# States
MOTOR_GLOBALS_POS = {}
MOTOR_CURRENT_ROTATION ={}
LAST_POS = {}
CUMULATIVE_STEPS = {}

# Threading settings
print_lock = threading.Lock()
monitor_stop = threading.Event()
comm_lock = threading.Lock()
state_lock = threading.Lock()

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

# -- Rotation tracking ---

def _init_motor_rotation_state(sid: int, pos_raw: int, user_rot: int):
    pos_mod = pos_raw % TICKS_PER_TURN
    with state_lock:
        LAST_POS[sid] = pos_mod
        MOTOR_CURRENT_ROTATION[sid] = int(user_rot)
        CUMULATIVE_STEPS[sid] = MOTOR_CURRENT_ROTATION[sid] *TICKS_PER_TURN + pos_mod


def update_rotation(sid: int, pos_raw: int):
    pos_mod = pos_raw % TICKS_PER_TURN
    with state_lock:
        last = LAST_POS[sid]

        delta = (pos_mod - last + HALF_TURN) % TICKS_PER_TURN - HALF_TURN
        CUMULATIVE_STEPS[sid] += delta

        LAST_POS[sid] = (last + delta) % TICKS_PER_TURN
        new_rot = int(CUMULATIVE_STEPS[sid] // TICKS_PER_TURN)
        if new_rot != MOTOR_CURRENT_ROTATION[sid]:
            MOTOR_CURRENT_ROTATION[sid] = new_rot



for sid in MOTOR_IDS:

    # --- Enable position mode ---
    result, error = packetHandler.write1ByteTxRx(sid, STS_MODE, 3)
    if result != COMM_SUCCESS:
        print(f"ID {sid}: {packetHandler.getTxRxResult(result)}")
    elif error != 0:
        print(f"ID {sid}: {packetHandler.getRxPacketError(error)}")
    else:
        print(f"ID {sid}: Position mode enabled.", )
    
   
    # --- Set limits ---

    # Turn of the limits by setting them both to 0:
    # (Still can only tunr +- 8 turns from midpoint (start point))

    sts_comm_result, sts_error= packetHandler.Write2Byte(sid, 0x09, 0) # Minimum angle
    # print(packetHandler.getTxRxResult(sts_comm_result))
    sts_comm_result, sts_error= packetHandler.Write2Byte(sid, 0x0b, 0) # Maxium angle
    # print(packetHandler.getTxRxResult(sts_comm_result))

    # Read the limits to verify:

    result, sts_comm_result, sts_error= packetHandler.Read2Byte(sid, 0x09) # Minimum angle
    # print(packetHandler.getTxRxResult(sts_comm_result),  f"ID {sid}: Minimum angle: {result}")
    print(f"ID {sid}: Minimum angle: {result}")

    result, sts_comm_result, sts_error= packetHandler.Read2Byte(sid, 0x0b) # Maxium angle
    # print(packetHandler.getTxRxResult(sts_comm_result), f"ID {sid}: Maximum angle: {result}")
    print(f"ID {sid}: Maximum angle: {result}")

    # --- Indivual Motor Global Positions ---

    pos, sts_comm_result, sts_error = packetHandler.ReadPos(sid)
    if sts_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(sts_comm_result))
    else:
        MOTOR_GLOBALS_POS[sid] = pos
        print(pos)
        
    
    # -- Indivual Rotations ---
    print("Input the current rotation of the motor. It should be 0, if not check: check previous output")
    try:
        rot = input(f"ID {sid}: Rotation:")
    except Exception:
        rot = 0
    _init_motor_rotation_state(sid, pos, rot)
    print(f"Set current rotation to: {MOTOR_CURRENT_ROTATION[sid]}")

print(f"Motor Global Dictionary: {MOTOR_GLOBALS_POS}")
print(f"Motor Indivual Rotation Disctionary: {MOTOR_CURRENT_ROTATION}")



input()



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

    update_rotation(ID, pos)

    with state_lock:
        rot = MOTOR_CURRENT_ROTATION[ID]
        global_pos = MOTOR_GLOBALS_POS.get(ID, 0)

    with print_lock:
        if sts_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(sts_comm_result))
        else:
            print(
                "[ID:%03d] Pos:%d Speed:%d Acc:%d (~%d steps/s^2) Current:%d global_position:%d byte67:%d Rotations:%d"
                % (ID, pos, speed, acc, acc * 100, current, global_pos, result, rot)
            )
        if sts_error != 0:
            print(packetHandler.getRxPacketError(sts_error))
    

def moveBySteps(ID, steps, speed, acc):

    with state_lock:
        MOTOR_GLOBALS_POS[ID] = MOTOR_GLOBALS_POS.get(ID, 0) + steps

    # MOTOR_GLOBALS_POS[ID] += steps
    # if steps<0:
    #     steps = -steps + 0x8000  # Convert to negative steps

    tx_steps = -steps + 0x8000 if steps < 0 else steps
    
    with comm_lock:
        sts_comm_result, sts_error = packetHandler.WritePosEx(ID, tx_steps, speed, acc)

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