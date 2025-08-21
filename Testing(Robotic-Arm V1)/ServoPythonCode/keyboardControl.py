#!/usr/bin/env python
import sys
import os
import time

# ---- Keyboard (cross-platform) ----
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode(errors='ignore')
else:
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# ---- STS SDK ----
sys.path.append("..")
from STservo_sdk import *
from STservo_sdk.sts import STS_ID

# ==== Settings ====
CANDIDATE_IDS   = [1,2,3,4,5,6]     # IDs to try
BAUDRATE        = 1_000_000         # Try 1,000,000 first; change if your servos use another
DEVICENAME      = 'COM3'            # e.g. Windows "COM3"; Linux "/dev/ttyUSB0"; macOS "/dev/tty.usbserial-*"
MOVE_SPEED      = 2400              # wheel speed unit: SDK/servo-specific (typically ~0–~~3000)
MOVE_ACC        = 50

# ==== Helpers ====
def print_comm_result(result, error, prefix=""):
    if result != COMM_SUCCESS:
        print(f"{prefix}[TxRxResult] {packetHandler.getTxRxResult(result)}")
    elif error != 0:
        print(f"{prefix}[RxPacketError] {packetHandler.getRxPacketError(error)}")

def send_speed(sid, speed, acc):
    """Write wheel speed with error handling."""
    res, err = packetHandler.WriteSpec(sid, speed, acc)
    print_comm_result(res, err, prefix=f"ID {sid}: ")

def try_ping_ids(ids):
    """Return list of IDs that actually respond."""
    found = []
    for sid in ids:
        # Some SDKs have Ping(), others ReadByte on model number; adapt if needed
        try:
            res, err = packetHandler.Ping(sid)
            if res == COMM_SUCCESS and err == 0:
                found.append(sid)
            else:
                print_comm_result(res, err, prefix=f"Ping {sid}: ")
        except Exception as e:
            print(f"Ping {sid} exception: {e}")
        time.sleep(0.01)
    return found

# ==== Open port ====
portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)

if not portHandler.openPort():
    print("❌ Failed to open port. Check DEVICENAME.")
    sys.exit(1)

# Small delay helps some adapters settle
time.sleep(0.2)

if not portHandler.setBaudRate(BAUDRATE):
    print("❌ Failed to set baudrate. Try 115200 or confirm your servo's setting.")
    portHandler.closePort()
    sys.exit(1)

print(f"✅ Port {DEVICENAME} open @ {BAUDRATE} bps")
time.sleep(0.2)

# ==== Detect IDs ====
detected_ids = try_ping_ids(CANDIDATE_IDS)
if not detected_ids:
    print("❌ No servos responded to ping. Check power, GND, half-duplex wiring, and baudrate.")
    portHandler.closePort()
    sys.exit(1)

print(f"🔎 Detected servos: {detected_ids}")

# ==== Enable wheel mode on detected ====
for sid in detected_ids:
    res, err = packetHandler.WheelMode(sid)
    if res == COMM_SUCCESS and err == 0:
        print(f"ID {sid}: Wheel mode enabled.")
    else:
        print_comm_result(res, err, prefix=f"WheelMode {sid}: ")

# ==== Key map for up to 6 IDs ====
# Map pairs of keys (CW/CCW) to the nth detected ID
pairs = [
    ('q', 'a'),  # for detected_ids[0]
    ('w', 's'),  # for detected_ids[1]
    ('e', 'd'),  # for detected_ids[2]
    ('r', 'f'),  # for detected_ids[3]
    ('t', 'g'),  # for detected_ids[4]
    ('y', 'h'),  # for detected_ids[5]
]

print("\nControls:")
for idx, (cw, ccw) in enumerate(pairs):
    if idx < len(detected_ids):
        print(f"  {cw}/{ccw}  → ID {detected_ids[idx]} CW/CCW")
print("  [Space] Stop all | [Esc] Quit\n")

# ==== Main loop ====
try:
    while True:
        key = getch()
        if not key:
            continue

        if key == chr(0x1b):  # ESC
            break

        if key == ' ':
            print("⏹ Stop All")
            for sid in detected_ids:
                send_speed(sid, 0, MOVE_ACC)
            continue

        handled = False
        for idx, (cw, ccw) in enumerate(pairs):
            if idx >= len(detected_ids):
                break
            sid = detected_ids[idx]
            if key.lower() == cw:
                print(f"ID {sid}: Clockwise")
                send_speed(sid, MOVE_SPEED, MOVE_ACC)
                handled = True
                break
            elif key.lower() == ccw:
                print(f"ID {sid}: Counter-clockwise")
                send_speed(sid, -MOVE_SPEED, MOVE_ACC)
                handled = True
                break

        if not handled:
            print(f"⏭  Unknown key '{repr(key)}' — no action taken.")

finally:
    # Stop motors and close cleanly
    for sid in detected_ids:
        try:
            packetHandler.WriteSpec(sid, 0, MOVE_ACC)
        except Exception:
            pass
    portHandler.closePort()
    if os.name != 'nt':
        # restore terminal state if on POSIX
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print("✅ Port closed. Motors stopped. Bye.")
