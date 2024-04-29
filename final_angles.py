import numpy as np
import math
import csv

def to_azimuth_elevation_with_position_array(roll_array, pitch_array, yaw_array, observer_position, x_position, y_position, z_position):
    azimuths = []
    elevations = []

    for i in range(len(roll_array)):
        roll = math.radians(roll_array[i])
        pitch = math.radians(pitch_array[i])
        yaw = math.radians(yaw_array[i])

        delta_x = x_position[i] - observer_position[0]
        delta_y = y_position[i] - observer_position[1]
        delta_z = z_position[i] - observer_position[2]

        rotated_x = delta_x * (math.cos(pitch) * math.cos(yaw)) + \
                    delta_y * (math.cos(pitch) * math.sin(yaw)) - \
                    delta_z * math.sin(pitch)
        
        rotated_y = delta_x * (math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw)) + \
                    delta_y * (math.sin(roll) * math.sin(pitch) * math.sin(yaw) + math.cos(roll) * math.cos(yaw)) + \
                    delta_z * (math.sin(roll) * math.cos(pitch))
        
        rotated_z = delta_x * (math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw)) + \
                    delta_y * (math.cos(roll) * math.sin(pitch) * math.sin(yaw) - math.sin(roll) * math.cos(yaw)) + \
                    delta_z * (math.cos(roll) * math.cos(pitch))

        azimuth = math.atan2(rotated_y, rotated_x)
        elevation = math.atan2(rotated_z, math.sqrt(rotated_x ** 2 + rotated_y ** 2))

        azimuths.append(math.degrees(azimuth))
        elevations.append(math.degrees(elevation))

    return azimuths, elevations

# Example usage:
