#!/usr/bin/env python

import sys
import os
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Platform-specific getch
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import tty, termios
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# STServo SDK import
sys.path.append("..")
from STservo_sdk import *

# === Config ===
motor_id = 1
DEVICENAME = 'COM6'
BAUDRATE = 1000000
TICKS_PER_TURN = 4096
STS_MOVING_SPEED = 1200
STS_MOVING_ACC = 50

# === Port Setup ===
portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)

if not portHandler.openPort():
    print("❌ Failed to open port.")
    sys.exit(1)

if not portHandler.setBaudRate(BAUDRATE):
    print("❌ Failed to set baudrate.")
    sys.exit(1)

# === Motor Mode Setup ===
res, err = packetHandler.write1ByteTxRx(motor_id, STS_MODE, 0)
if res != COMM_SUCCESS or err != 0:
    print("❌ Failed to set mode")
else:
    print(f"✅ Motor {motor_id} set to position mode")

# === Live Plot Config ===
plot_duration = 20
sample_interval = 0.1
max_points = int(plot_duration / sample_interval)

live_data = {
    "Time": deque(maxlen=max_points),
    "Speed": deque(maxlen=max_points),
    "Load (%)": deque(maxlen=max_points),
    "Current (mA)": deque(maxlen=max_points),
    "Torque (%)": deque(maxlen=max_points),
    "Torque (calculated)": deque(maxlen=max_points)
}

plot_options = list(live_data.keys())[1:]
print("\nSelect a metric to display live:")
for i, opt in enumerate(plot_options):
    print(f"{i+1}. {opt}")

while True:
    try:
        selection = int(input("Enter choice number: "))
        if 1 <= selection <= len(plot_options):
            selected_plot = plot_options[selection - 1]
            break
        else:
            print("Invalid choice.")
    except ValueError:
        print("Enter a number.")

# === Matplotlib Setup ===
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_title(f"Live {selected_plot} Plot")
ax.set_xlabel("Time (s)")
ax.set_ylabel(selected_plot)
ax.grid(True)
start_time = time.time()

# === Live Data Update ===
def update_data():
    now = time.time() - start_time
    speed, _, _ = packetHandler.ReadSpeed(motor_id)
    load, _, _ = packetHandler.ReadLoad(motor_id)
    current, _, _ = packetHandler.ReadCurrent(motor_id)
    torque = load
    torque_calc = (load / 100.0) * 1.0

    live_data["Time"].append(now)
    live_data["Speed"].append(speed)
    live_data["Load (%)"].append(load)
    live_data["Current (mA)"].append(current)
    live_data["Torque (%)"].append(torque)
    live_data["Torque (calculated)"].append(torque_calc)

# === Animation Frame Update ===
def animate(frame):
    update_data()
    times = list(live_data["Time"])
    values = list(live_data[selected_plot])
    line.set_data(times, values)
    ax.relim()
    ax.autoscale_view()
    return line,

ani = animation.FuncAnimation(fig, animate, interval=sample_interval * 1000, blit=True)
plt.tight_layout()
plt.show()

portHandler.closePort()
print("✅ Port closed. Program ended.")
