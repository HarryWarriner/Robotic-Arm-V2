import lcm
import time

from lcm_msgs import franka_joint_target_t, gripper_command_t, keyboard_state_t

lc = lcm.LCM()

# === Test franka_joint_target_t ===
joint_target = franka_joint_target_t()
joint_target.position = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
lc.publish("FRANKA_JOINT_TARGET", joint_target.encode())
print("Sent franka_joint_target_t")

# === Test gripper_command_t ===
gripper_cmd = gripper_command_t()
gripper_cmd.open = True
lc.publish("GRIPPER_COMMAND", gripper_cmd.encode())
print("Sent gripper_command_t")

# === Test keyboard_state_t ===
keyboard_state = keyboard_state_t()
keyboard_state.nkeys = 3
keyboard_state.label = ["Up", "Down", "Enter"]
keyboard_state.state = [True, False, True]
lc.publish("KEYBOARD_STATE", keyboard_state.encode())
print("Sent keyboard_state_t")

# Keep LCM alive briefly to ensure delivery
time.sleep(0.1)
