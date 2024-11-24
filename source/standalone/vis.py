import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define parameters for the synthetic inverted conical structure
angle_of_repose_deg = 30  # Expected angle of repose in degrees
angle_of_repose_rad = np.radians(angle_of_repose_deg)
height = 1.0  # Height of the cone
radius = height / np.tan(angle_of_repose_rad)  # Calculate radius based on the angle

# Generate points for the inverted conical pile with points inside the cone
num_points = 10000
z_vals = np.random.uniform(0, height, num_points)  # Random z values within cone height
max_radius_at_z = (height - z_vals) * np.tan(angle_of_repose_rad)  # Radius decreases as z decreases
r_vals = max_radius_at_z * np.sqrt(np.random.uniform(0.9, 1, num_points))  # Constrain to outer layers

# Convert cylindrical (r, θ, z) coordinates to Cartesian (x, y, z)
theta_vals = np.random.uniform(0, 2 * np.pi, num_points)
x_vals = r_vals * np.cos(theta_vals)
y_vals = r_vals * np.sin(theta_vals)

# Combine points for an inverted cone
inverted_points = np.vstack((x_vals, y_vals, z_vals)).T

# Visualize the synthetic inverted conical structure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(inverted_points[:, 0], inverted_points[:, 1], inverted_points[:, 2], color='purple', s=1, label='Particle Points')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title(f"Synthetic Inverted Conical Pile with Angle of Repose: {angle_of_repose_deg}°")
plt.legend()
plt.show()

# Fitting planes for base and slope to calculate the angle of repose
# Fit a plane to the top (wide end) region at z = height
base_points = inverted_points[inverted_points[:, 2] > height - 0.01]  # Narrow range at the top for flat base
slope_points = inverted_points[(inverted_points[:, 2] <= height - 0.05) & (r_vals >= 0.95 * max_radius_at_z)]  # Only outer slope points

# Use least squares to fit a plane to the base and slope points
def fit_plane(points):
    A = np.c_[points[:, 0], points[:, 1], np.ones(points.shape[0])]
    C, _, _, _ = np.linalg.lstsq(A, points[:, 2], rcond=None)  # Plane coefficients
    normal = np.array([-C[0], -C[1], 1])
    return normal / np.linalg.norm(normal)

base_normal = fit_plane(base_points)
slope_normal = fit_plane(slope_points)

# Calculate the angle between the base and slope planes
cos_theta = np.dot(base_normal, slope_normal)
calculated_angle_of_repose = np.degrees(np.arccos(cos_theta))

print(f"Calculated Angle of Repose: {calculated_angle_of_repose:.2f} degrees")
