import numpy as np
import matplotlib.pyplot as plt

# === Robot Geometry (link lengths in arbitrary units) ===
l1 = 2.0  # base to shoulder (vertical)
l2 = 4.0  # upper arm
l3 = 3.0  # forearm
l4 = 1.0  # wrist to end-effector (tool length)

# === IK Function for 5DOF arm with fixed end-effector pitch ===
def ik_5dof(x, y, z, pitch_deg, roll_deg):
    pitch = np.radians(pitch_deg)
    roll = np.radians(roll_deg)

    # Joint 1: Base rotation
    theta1 = np.arctan2(y, x)

    # Project target to base frame
    r = np.sqrt(x**2 + y**2)
    wx = r - l4 * np.cos(pitch)
    wz = z - l1 - l4 * np.sin(pitch)

    # Distance to wrist center
    D_sq = wx**2 + wz**2
    D = np.sqrt(D_sq)

    # Check reachability
    cos_theta3 = (D_sq - l2**2 - l3**2) / (2 * l2 * l3)
    if np.abs(cos_theta3) > 1:
        return None  # Unreachable

    theta3 = np.arccos(cos_theta3)  # elbow-down
    k1 = l2 + l3 * np.cos(theta3)
    k2 = l3 * np.sin(theta3)
    theta2 = np.arctan2(wz, wx) - np.arctan2(k2, k1)

    # Wrist pitch (keep constant orientation)
    theta4 = pitch - theta2 - theta3

    # Wrist roll (assume independent)
    theta5 = roll

    return [theta1, theta2, theta3, theta4, theta5]

# === Forward Kinematics for plotting ===
def forward_kinematics(theta1, theta2, theta3, theta4):
    # Shoulder point
    x0, y0, z0 = 0, 0, 0
    x1, y1, z1 = 0, 0, l1

    # Project into rotated XY plane by theta1
    a = theta1
    x2 = x1 + l2 * np.cos(theta2) * np.cos(a)
    y2 = y1 + l2 * np.cos(theta2) * np.sin(a)
    z2 = z1 + l2 * np.sin(theta2)

    x3 = x2 + l3 * np.cos(theta2 + theta3) * np.cos(a)
    y3 = y2 + l3 * np.cos(theta2 + theta3) * np.sin(a)
    z3 = z2 + l3 * np.sin(theta2 + theta3)

    x4 = x3 + l4 * np.cos(theta2 + theta3 + theta4) * np.cos(a)
    y4 = y3 + l4 * np.cos(theta2 + theta3 + theta4) * np.sin(a)
    z4 = z3 + l4 * np.sin(theta2 + theta3 + theta4)

    return np.array([[x0, y0, z0],
                     [x1, y1, z1],
                     [x2, y2, z2],
                     [x3, y3, z3],
                     [x4, y4, z4]])

# === Test target ===
target_x = 4
target_y = 2
target_z = 5
tool_pitch = 45  # degrees
tool_roll = 30   # degrees

# === Solve IK and plot ===
ik_result = ik_5dof(target_x, target_y, target_z, tool_pitch, tool_roll)

if ik_result:
    joints = forward_kinematics(*ik_result[:-1])
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(joints[:, 0], joints[:, 1], joints[:, 2], 'o-', lw=3)
    ax.scatter(target_x, target_y, target_z, c='r', label='Target')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('5DOF Arm IK Solution')
    ax.legend()
    ax.set_box_aspect([1,1,1])
    plt.show()
else:
    print("❌ Target unreachable with current configuration.")
