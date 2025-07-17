import numpy as np

# Link lengths in mm
L1 = 228.5
L2 = 228.5
L3 = 242.5

# Angles in degrees
theta1_deg = 67.7
theta2_deg = 148
theta3_deg = -42

# Convert to radians
theta1 = np.radians(theta1_deg)
theta2 = np.radians(theta2_deg)
theta3 = np.radians(theta3_deg)

print("Theta in rads:", theta1, theta2, theta3)

# Cumulative angles
theta12 = theta1 + theta2
theta123 = theta12 + theta3

print("Theta 12", theta12)
print("Theta 123", theta123)
# Position after first link
x1 = L1 * np.cos(theta1)
y1 = L1 * np.sin(theta1)

# Position after second link
x2 = L2 * np.cos(theta12)
y2 = L2 * np.sin(theta12)

# Position after third link (end effector)
x3 = L3 * np.cos(theta123)
y3 = L3 * np.sin(theta123)

# Final position (sum of all)
x_final = x1 + x2 + x3
y_final = y1 + y2 + y3
print(x1, x2, x3)
print(y1, y2, y3)
print(x_final, y_final)
