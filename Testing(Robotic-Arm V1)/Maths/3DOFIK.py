import numpy as np
import matplotlib.pyplot as plt

# Link lengths in cm
l1 = 22.65
l2 = 22.65
l3 = 24.25 # End effector offset

def analytical_ik_3dof(x, y, phi=0.0):
    """
    x, y = desired end-effector position (in cm)
    phi = desired end-effector orientation (in radians)

    Returns θ1, θ2, θ3 in radians or None if unreachable
    """
    # Compute wrist position (subtract end effector offset)
    wx = x - l3 * np.cos(phi)
    wy = y - l3 * np.sin(phi)

    # Solve for θ1 and θ2 using 2-link arm IK
    r_sq = wx**2 + wy**2
    cos_theta2 = (r_sq - l1**2 - l2**2) / (2 * l1 * l2)

    if abs(cos_theta2) > 1:
        return None  # unreachable

    theta2 = np.arccos(cos_theta2)
    k1 = l1 + l2 * np.cos(theta2)
    k2 = l2 * np.sin(theta2)
    theta1 = np.arctan2(wy, wx) - np.arctan2(k2, k1)

    # θ3 aligns the end effector
    theta3 = phi - theta1 - theta2

    return theta1, theta2, theta3

def forward_kinematics_3dof(theta1, theta2, theta3):
    """
    Returns joint positions: shoulder, elbow, wrist, end-effector
    """
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    x3 = x2 + l3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + l3 * np.sin(theta1 + theta2 + theta3)
    return [(0, 0), (x1, y1), (x2, y2), (x3, y3)]

# Plot setup
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-80, 80)
ax.set_ylim(-10, 80)
ax.grid(True)
plt.title("3DOF Planar IK — Click to Move Target")

# Initial target and orientation
target = [15, 15]
target_phi = -np.pi/2  # Radians

# Plot elements
line, = ax.plot([], [], 'o-', lw=4, color='blue')
target_dot, = ax.plot([], [], 'rx', markersize=12, mew=2)
text = ax.text(-38, 35, "", fontsize=12)

def update_arm(event):
    global target
    if event.inaxes != ax:
        return

    target = [event.xdata, event.ydata]
    ik_result = analytical_ik_3dof(*target, phi=target_phi)

    if ik_result is None:
        text.set_text("Unreachable")
        line.set_data([], [])
    else:
        theta1, theta2, theta3 = ik_result
        joints = forward_kinematics_3dof(theta1, theta2, theta3)
        xs, ys = zip(*joints)
        line.set_data(xs, ys)
        text.set_text(
            f"θ₁={np.degrees(theta1):.1f}°, θ₂={np.degrees(theta2):.1f}°, θ₃={np.degrees(theta3):.1f}°"
        )

    target_dot.set_data([target[0]], [target[1]])
    fig.canvas.draw()

fig.canvas.mpl_connect('button_press_event', update_arm)
update_arm(type('dummy', (object,), {'xdata': target[0], 'ydata': target[1], 'inaxes': ax})())

plt.show()
