#!/usr/bin/env python
#
# *********     Gen Write Example      *********
#
#
# Available STServo model on this example : All models using Protocol STS
# This example is tested with a STServo and an URT
#

import sys
import time
import os


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
from STservo_sdk import *                      # Uses STServo SDK library

# Default setting
STS_ID                      = 1                 # STServo ID : 1
BAUDRATE                    = 1000000           # STServo default baudrate : 1000000
DEVICENAME                  = 'COM6'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
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


result, sts_comm_result, sts_error= packetHandler.Read2Byte(STS_ID, 0x09) # Minimum angle
print(packetHandler.getTxRxResult(sts_comm_result), "Minimum angle:", result)

result, sts_comm_result, sts_error= packetHandler.Read2Byte(STS_ID, 0x0b) # Maxium angle
print(packetHandler.getTxRxResult(sts_comm_result), "Maximum angle:", result)

sts_comm_result, sts_error= packetHandler.Write2Byte(STS_ID, 0x09, 0) # Minimum angle
print(packetHandler.getTxRxResult(sts_comm_result))
sts_comm_result, sts_error= packetHandler.Write2Byte(STS_ID, 0x0b, 0) # Maxium angle
print(packetHandler.getTxRxResult(sts_comm_result))

sts_comm_result, sts_error= packetHandler.Write2Byte(STS_ID, 33, 3) # Step servo mode
print(packetHandler.getTxRxResult(sts_comm_result))


result, sts_comm_result, sts_error= packetHandler.Read2Byte(STS_ID, 0x09) # Minimum angle
print(packetHandler.getTxRxResult(sts_comm_result), "Minimum angle:", result)

result, sts_comm_result, sts_error= packetHandler.Read2Byte(STS_ID, 0x0b) # Maxium angle
print(packetHandler.getTxRxResult(sts_comm_result), "Maximum angle:", result)

global_position = 0

def showPosition():
  result, sts_comm_result, sts_error= packetHandler.Read2Byte(STS_ID, 67) # Unknown
  print("Unknown:", result)  
  # Read STServo present position
  sts_present_position, sts_present_speed, sts_comm_result, sts_error = packetHandler.ReadPosSpeed(STS_ID)
  if sts_comm_result != COMM_SUCCESS:
      print(packetHandler.getTxRxResult(sts_comm_result))
  else:
      print("[ID:%03d] PresPos:%d PresSpd:%d global_position:%d" % (STS_ID, sts_present_position, sts_present_speed, global_position))
  if sts_error != 0:
      print(packetHandler.getRxPacketError(sts_error))
    
      

def getCurrentPosition():
    #
    sts_present_position, sts_present_speed, sts_comm_result, sts_error = packetHandler.ReadPosSpeed(STS_ID)
    if sts_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(sts_comm_result))
    elif sts_error != 0:
        print(packetHandler.getRxPacketError(sts_error))
    else:
        print("[ID:%03d] PresPos:%d PresSpd:%d" % (STS_ID, sts_present_position, sts_present_speed))
        # Return the current position
        return sts_present_position
    return None

def moveBySteps(steps):
    global global_position
    print("Moving by steps:", steps)
    global_position += steps
    if steps<0:
        steps = -steps + 0x8000  # Convert to negative steps
    
    sts_comm_result, sts_error = packetHandler.WritePosEx(STS_ID, steps, 1000,100)
    if sts_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(sts_comm_result))
    elif sts_error != 0:
        print("%s" % packetHandler.getRxPacketError(sts_error))
        
    while True:
      result, sts_comm_result, sts_error= packetHandler.ReadByte(STS_ID, 0x42) # Is it moving?
      if sts_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(sts_comm_result))
      elif sts_error != 0:
        print("%s" % packetHandler.getRxPacketError(sts_error))
      else:
        print("[ID:%03d] Moving status: %d" % (STS_ID, result))
        if result==0:
          print("Servo stopped moving")
          break
        else:
          time.sleep(0.1)
        
    
        
        
def moveToPosition(position):
    print("Moving to position:", position, " from global_position:", global_position)
    moveBySteps(position - global_position)


while 1:
    showPosition()
    print("Press i-info, p-pos, -zero, a- -100, b +100, q-quit")
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
        showPosition()
    elif ch == '0':
      moveToPosition(0)
    elif ch == '-':
      result, sts_comm_result, sts_error= packetHandler.Read2Byte(STS_ID, 67) # Unknown
      print("Current:", result)  
      moveBySteps(-result) # Move to zero position
    elif ch == 'a' or ch=='b':
      if ch=='a':
          moveBySteps(-1024)
      else:
          moveBySteps(+1024)
# Close port
portHandler.closePort()