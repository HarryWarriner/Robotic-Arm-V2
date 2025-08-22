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
MOTOR_IDS = [1,2]
GEAR_RATIOS = {1:1 ,2:3}
BAUDRATE = 1000000
DEVICENAME  = 'COM5'
 # e.g. Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
SPEED = 1200
ACC = 50

# Ticks
TICKS_PER_TURN = 4096
HALF_TURN = TICKS_PER_TURN // 2

# Target controller limits (WritePosEx uses 16-bit w/ sign trick here)
CHUNK_LIMIT = 30000       # keep under 0x7FFF

# States
MOTOR_GLOBALS_STEPS = {}
MOTOR_CURRENT_ROTATION ={}
LAST_POS = {}
LAST_ABS67 = {}


# Movement
TARGET_STEPS = {}
TARGET_ANGLES = {}



# Threading settings
print_lock = threading.Lock()
comm_lock = threading.Lock()
state_lock = threading.Lock()

# Worker Threads
workers_lock = threading.Lock()
workers = set()

def spawn(name, target, *args, **kwargs):
    """Start a daemon worker thread for a key action."""
    def runner():
        try:
            target(*args, **kwargs)
        except Exception as e:
            with print_lock:
                print(f"[{name}] error:", e)
        finally:
            with workers_lock:
                workers.discard(threading.current_thread())
    t = threading.Thread(target=runner, name=name, daemon=True)
    with workers_lock:
        workers.add(t)
    t.start()
    return t

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

def _init_motor_rotation_state(sid: int, abs67: int, user_rot: int):
    """Initialize rotation state from absolute 16-bit reg67."""
    abs67 &= 0xFFFF
    with state_lock:
        LAST_ABS67[sid] = abs67
        MOTOR_CURRENT_ROTATION[sid] = int(user_rot)
        MOTOR_GLOBALS_STEPS[sid] = MOTOR_CURRENT_ROTATION[sid] * TICKS_PER_TURN + (abs67 % TICKS_PER_TURN)
        TARGET_STEPS[sid] = MOTOR_GLOBALS_STEPS[sid]
        TARGET_ANGLES[sid] = 0
        



def update_rotation_from_abs67(sid: int, abs67: int):
    """Update rotation using wrap-aware delta of reg67 (16-bit)."""
    abs67 &= 0xFFFF
    with state_lock:
        prev = LAST_ABS67.get(sid, abs67)
        # signed 16-bit wrap diff: range [-32768, +32767]
        delta = ((abs67 - prev + 32768) % 65536) - 32768
        MOTOR_GLOBALS_STEPS[sid] += delta
        LAST_ABS67[sid] = abs67
        MOTOR_CURRENT_ROTATION[sid] = int(MOTOR_GLOBALS_STEPS[sid] // TICKS_PER_TURN)
        return MOTOR_CURRENT_ROTATION[sid]
    
def read_abs67(sid: int):
    with comm_lock:
        val, comm, err = packetHandler.Read2Byte(sid, 67)
    if comm == COMM_SUCCESS and err == 0:
        update_rotation_from_abs67(sid, val)
    else:
        with print_lock:
            if comm != COMM_SUCCESS: print(packetHandler.getTxRxResult(comm))
            if err != 0: print(packetHandler.getRxPacketError(err))
    return val, comm, err



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

    # # --- Indivual Motor Global Positions ---

    # pos, sts_comm_result, sts_error = packetHandler.ReadPos(sid)
    # if sts_comm_result != COMM_SUCCESS:
    #     print(packetHandler.getTxRxResult(sts_comm_result))
    # else:
    #     MOTOR_GLOBALS_POS[sid] = pos
    #     print(pos)

    # --- Individual Motor Absolute (reg67) ---
    abs67, sts_comm_result, sts_error = packetHandler.Read2Byte(sid, 67)
    if sts_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(sts_comm_result))
    else:
        print(f"ID {sid}: reg67 initial = {abs67}")

    # -- Individual Rotations ---
    print("Input the current rotation of the motor. It should be 0, if not check: check previous output")
    try:
        rot_str = input(f"ID {sid}: Rotation:").strip()
        rot = int(rot_str) if rot_str else 0
    except Exception:
        rot = 0
    _init_motor_rotation_state(sid, abs67, rot)
    print(f"Set current rotation to: {MOTOR_CURRENT_ROTATION[sid]}")

    # --- Indivual Motor Global Positions ---
    # MOTOR_GLOBALS_POS[sid] = TICKS_PER_TURN * rot + (abs67 % TICKS_PER_TURN)
    # print(TICKS_PER_TURN * rot + (abs67 % TICKS_PER_TURN))
    # print(abs67)
    # print(rot)
    # print(abs67 % TICKS_PER_TURN)


print(f"Motor Target Steps Disctionary: {TARGET_STEPS}")
print(f"Target Anges: {TARGET_ANGLES}")
print(f"Motor Global Steps Dictionary: {MOTOR_GLOBALS_STEPS}")
print(f"Motor Indivual Rotation Disctionary: {MOTOR_CURRENT_ROTATION}")





def showPosition(ID):
    # with comm_lock:
    #     result, sts_comm_result, sts_error= packetHandler.Read2Byte(ID, 67) # Unknown
    # with print_lock:
    #     print("Code Absolute Position:", result)  

    with comm_lock:
        # Read STServo present position, speed, acceleration, current
        pos, speed, acc, current, sts_comm_result, sts_error = packetHandler.ReadPosSpeedAccCurrent(ID)
        result67, sts_comm_result2, sts_error2 = packetHandler.Read2Byte(ID, 67)

    update_rotation_from_abs67(ID, result67)

    with state_lock:
        rot = MOTOR_CURRENT_ROTATION[ID]
        global_steps = MOTOR_GLOBALS_STEPS.get(ID, 0)
        tgt = TARGET_STEPS.get(ID, global_steps)
        err = tgt - global_steps

    with print_lock:
        if sts_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(sts_comm_result))
        else:
            print(
                "[ID:%03d] Speed:%d Acc:%d (~%d steps/s^2) Current:%d global_steps:%d byte67:%d Rot:%d Target:%d Err:%d"
                % (ID, speed, acc, acc * 100, current, global_steps, result67, rot, tgt, err)
            )
        if sts_error != 0:
            print(packetHandler.getRxPacketError(sts_error))
        if sts_comm_result2 != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(sts_comm_result2))
        if sts_error2 != 0:
            print(packetHandler.getRxPacketError(sts_error2))
    

# def moveBySteps(ID, steps, speed, acc):

#     # with state_lock:
#         # MOTOR_GLOBALS_POS[ID] = MOTOR_GLOBALS_POS.get(ID, 0) + steps

#     # MOTOR_GLOBALS_POS[ID] += steps
#     # if steps<0:
#     #     steps = -steps + 0x8000  # Convert to negative steps

#     tx_steps = -steps + 0x8000 if steps < 0 else steps
    
#     with comm_lock:
#         sts_comm_result, sts_error = packetHandler.WritePosEx(ID, tx_steps, speed, acc)

#     if sts_comm_result != COMM_SUCCESS:
#         with print_lock:
#             print("%s" % packetHandler.getTxRxResult(sts_comm_result))
#         return
#     if sts_error != 0:
#         with print_lock:
#             print("%s" % packetHandler.getRxPacketError(sts_error))
#         return
    
#     # poll moving flag
#     while True:
#         with comm_lock:
#             moving, comm2, err2 = packetHandler.ReadByte(ID, 0x42)  # STS_MOVING
#         if comm2 != COMM_SUCCESS:
#             with print_lock:
#                 print(packetHandler.getTxRxResult(comm2))
#             break
#         if err2 != 0:
#             with print_lock:
#                 print(packetHandler.getRxPacketError(err2))
#             break
#         if moving == 0:
#             break
#         time.sleep(0.1)
def moveBySteps(ID, steps, speed, acc):
    """One-shot relative move by steps (handles sign encoding), blocks until stop."""
    # Encode negative as +0x8000 trick used by this SDK
    tx_steps = -steps + 0x8000 if steps < 0 else steps
    with comm_lock:
        sts_comm_result, sts_error = packetHandler.WritePosEx(ID, tx_steps, speed, acc)
    if sts_comm_result != COMM_SUCCESS:
        with print_lock: print(packetHandler.getTxRxResult(sts_comm_result))
        return
    if sts_error != 0:
        with print_lock: print(packetHandler.getRxPacketError(sts_error))
        return
    # poll moving flag
    while True:
        with comm_lock:
            moving, comm2, err2 = packetHandler.ReadByte(ID, 0x42)  # STS_MOVING
        if comm2 != COMM_SUCCESS:
            with print_lock: print(packetHandler.getTxRxResult(comm2))
            break
        if err2 != 0:
            with print_lock: print(packetHandler.getRxPacketError(err2))
            break
        if moving == 0:
            break
        time.sleep(0.05)
    print(TARGET_STEPS, MOTOR_GLOBALS_STEPS)

# ----------------- Position controller (NEW) -----------------
def position_controller(ID):
   
    while True:
        difference = TARGET_STEPS[ID] - MOTOR_GLOBALS_STEPS[ID]
        moveBySteps(ID, difference, 1200, 50)
        read_abs67(ID)
       
def dump_registers():
    for index in range(0, 70):
        with comm_lock:
            result, sts_comm_result, sts_error = packetHandler.ReadByte(STS_ID, index)
        with print_lock:
            if sts_comm_result != COMM_SUCCESS:
                print(packetHandler.getTxRxResult(sts_comm_result))
            else:
                print("[ID:%03d] Reg[%02d] = %d" % (STS_ID, index, result))
            if sts_error != 0:
                print(packetHandler.getRxPacketError(sts_error))

def goto_zero():
    for sid in MOTOR_IDS:
        with comm_lock:
            result, sts_comm_result, sts_error = packetHandler.Read2Byte(sid, 67)
        with print_lock:
            print("Current:", result)
        moveBySteps(sid, -result, SPEED, ACC)

def show_all_positions():
    print("Motor Positions:")
    for sid in MOTOR_IDS:
        showPosition(sid)

# -- Angles ---

def update_target_angles(ID, angle):
    steps = int(round(angle * (TICKS_PER_TURN * float(GEAR_RATIOS[ID]) / 360)))
    with state_lock:
        TARGET_ANGLES[ID] += angle   
        TARGET_STEPS[ID] += steps
    with print_lock:
        print("Target Angles and Steps")
        print(f"[{ID}] +{angle}° -> Δsteps={steps}, total_angle={TARGET_ANGLES[ID]}°, target_steps={TARGET_STEPS[ID]}")

def update_target_steps(ID, delta_steps):
    with state_lock:
        TARGET_STEPS[ID] += int(delta_steps)
    with print_lock:
        print(f"[{ID}] Δtarget_steps={delta_steps} -> target_steps={TARGET_STEPS[ID]}")


# start controllers (NEW)
for sid in MOTOR_IDS:
    spawn(f"ctrl_{sid}", position_controller, sid)

# ------------- Key loop (each action on its own thread) -------------
try:
    while True:
        ch = getch()
        if ch == chr(0x1b):  # ESC
            break
        elif ch == 'i':
            spawn("dump_registers", dump_registers)
        elif ch == 'p':
            spawn("show_all_positions", show_all_positions)
        elif ch == '-':
            spawn("goto_zero", goto_zero)
        elif ch == 'q':
            # spawn("m0_minus", moveBySteps, MOTOR_IDS[0], -1024, SPEED, ACC)
            spawn("m0_minus", update_target_angles, MOTOR_IDS[0], +10)
        elif ch == 'a':
            spawn("m0_plus", moveBySteps, MOTOR_IDS[0], +1024, SPEED, ACC)
        elif ch == 'w':
            spawn("m1_minus", moveBySteps, MOTOR_IDS[1], -1024, SPEED, ACC)
        elif ch == 's':
            spawn("m1_plus", moveBySteps, MOTOR_IDS[1], +1024, SPEED, ACC)
        # else: ignore other keys
finally:
    # Give in-flight workers a brief chance to finish nicely
    with workers_lock:
        current_workers = list(workers)
    for t in current_workers:
        t.join(timeout=2.0)
    portHandler.closePort()
    print("Port closed. Bye.")