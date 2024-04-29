import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calibrate_magnetometer(data):
    """
    Calibrate magnetometer data.

    Parameters:
        data (numpy.ndarray): A numpy array of shape (N, 3) containing magnetometer measurements.

    Returns:
        numpy.ndarray: Calibrated magnetometer data of the same shape as input.
    """
    # Calculate offset and scale
    offset = np.mean(data, axis=0)
    scale = np.max(data, axis=0) - np.min(data, axis=0)

    # Calibrate data
    calibrated_data = (data - offset) / scale

    return calibrated_data

# Example magnetometer data (replace this with your actual data)
magnetometer_data = np.random.rand(10,3)  # Replace with your actual data

# Calibrate the magnetometer data
calibrated_data = calibrate_magnetometer(magnetometer_data)

# Plot raw magnetometer data
fig = plt.figure(figsize=(12, 6))

ax1 = fig.add_subplot(121, projection='3d')
ax1.scatter(magnetometer_data[:, 0], magnetometer_data[:, 1], magnetometer_data[:, 2], c='b', label='Raw Data')
ax1.set_title('Raw Magnetometer Data')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.legend()

# Plot calibrated magnetometer data
ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(calibrated_data[:, 0], calibrated_data[:, 1], calibrated_data[:, 2], c='r', label='Calibrated Data')
ax2.set_title('Calibrated Magnetometer Data')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
ax2.legend()

plt.tight_layout()
plt.show()
