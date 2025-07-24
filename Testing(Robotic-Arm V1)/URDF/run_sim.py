import pybullet as p
import pybullet_data
import time

# Start PyBullet GUI
p.connect(p.GUI)

# Set the simulation environment
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load a plane and the URDF robot
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("/SimpleArm/arm.urdf", basePosition=[0, 0, 0.2])

# Run the simulation
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

# Disconnect
p.disconnect()
