from ast import Constant
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from transforms3d import euler
from PIL import Image
import const
def update_plot1(frame, x, y, z, roll, pitch, yaw, vector_scale, ax):
    ax.cla()  # Clear the current plot
    
    # Plot position
    ax.scatter(x, y, z, color='r', label='Position')

    if x is not None and y is not None and z is not None:
        # Plot the points
        ax.scatter(x[frame], y[frame], z[frame], color='b', label='Points')

        # Calculate the midpoint of the cylinder
    
    # Define rotation matrix using Euler angles
    R = euler.euler2mat(np.deg2rad(roll[frame]), np.deg2rad(pitch[frame]), np.deg2rad(yaw[frame]), 'sxyz')
    
    # Define orientation vectors
    x_vec = R.dot(np.array([1, 0, 0]))
    y_vec = R.dot(np.array([0, 1, 0]))
    z_vec = R.dot(np.array([0, 0, 1]))
    
    # Scale orientation vectors
    x_vec *= 83
    y_vec *= 83
    z_vec *= 10
    
    # Plot orientation vectors
    ax.quiver(x[frame], y[frame], z[frame], x_vec[0], x_vec[1], x_vec[2], color='r')
    ax.quiver(x[frame], y[frame], z[frame], y_vec[0], y_vec[1], y_vec[2], color='g')
    ax.quiver(x[frame], y[frame], z[frame], z_vec[0], z_vec[1], z_vec[2], color='b')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Position and Orientation Plot')
    ax.legend()

def plot_animation_orient(x, y, z, roll, pitch, yaw, vector_scale):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    num_frames = len(x)

    anim = FuncAnimation(fig, update_plot1, frames=num_frames, fargs=(x, y, z, roll, pitch, yaw, vector_scale, ax))
    anim.save('position_orientation_orient.gif', writer='pillow', fps=100)

    plt.show()

def roll_yaw_pitch_to_spherical(roll, yaw, pitch, r=1.0):
    # Convert degrees to radians
    roll_rad = np.deg2rad(roll)
    yaw_rad = np.deg2rad(yaw)
    pitch_rad = np.deg2rad(pitch)
    
    # Calculate polar angle (θ)
    theta = np.pi / 2 - pitch_rad
    
    # Calculate azimuthal angle (φ)
    phi = yaw_rad
    
    return r, theta, phi

