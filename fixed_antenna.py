import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D




def plot_cylinder_and_points(x_cylinder, y_cylinder, z_cylinder, radius, height, x_points=None, y_points=None, z_points=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Create a cylinder
    z_cylinder_mesh = np.linspace(0, height, 100)
    theta = np.linspace(0, 2*np.pi, 100)
    Z_cylinder, Theta = np.meshgrid(z_cylinder_mesh, theta)
    X_cylinder = radius * np.cos(Theta)
    Y_cylinder = radius * np.sin(Theta)
    
    # Plot the cylinder
    ax.plot_surface(X_cylinder + x_cylinder, Y_cylinder + y_cylinder, Z_cylinder + z_cylinder, alpha=1)

    # Plot the cylinder center
    ax.scatter(x_cylinder, y_cylinder, z_cylinder, color='r', label='Cylinder Center')

    if x_points is not None and y_points is not None and z_points is not None:
        # Plot the points
        ax.scatter(x_points, y_points, z_points, color='b', label='Points')

        # Calculate the midpoint of the cylinder
        mid_x = x_cylinder
        mid_y = y_cylinder
        mid_z = z_cylinder + height / 2

        # Plot a line connecting the last point to the midpoint of the cylinder
        last_point_index = -1
        ax.plot([x_points[last_point_index], mid_x], [y_points[last_point_index], mid_y], [z_points[last_point_index], mid_z], color='g', linestyle='--')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Cylinder and Points')
    ax.legend()

    plt.show()


