import const
import filter_data
import Compute_distance
import Read_Data
from final_angles import to_azimuth_elevation_with_position_array
from fixed_antenna import  plot_cylinder_and_points
import geodetic_to_ltp
from kalman_filt import estimate_position, plot_distance
import matplotlib.pyplot as plt
from save_annimation_1 import plot_animation_orient
import csv
from saveanimation import plot_animation
ax=Read_Data.axf
ay=Read_Data.ayf
az=Read_Data.azf
dt=const.dt
latitude=Read_Data.latitude                                             
lonongitude=Read_Data.longitude
alttitude=Read_Data.altitude

lat,lon,alt=geodetic_to_ltp.geodetic_to_cartesian(latitude,lonongitude,alttitude)
x,y,z=Compute_distance.compute_distance_from_accleration(ax, ay, az)
# Example usage
dt = 0.01  # Time step
acc_x = Read_Data.axf  # Example accelerometer readings along x-axis
acc_y = Read_Data.ayf
acc_z = Read_Data.azf
gps_distances_x =lat  # Example GPS distance measurements
gps_distances_y =lon  # Example GPS distance measurements
gps_distances_z =alt  # Example GPS distance measurements
estimated_positions_x = estimate_position(acc_x, gps_distances_x, dt)
estimated_positions_y = estimate_position(acc_y, gps_distances_y, dt)
estimated_positions_z = estimate_position(acc_z, gps_distances_z, dt)
##saving position data. gps vs estimated

# Combine the arrays into a list of tuples (ax, ay, az)
acceleration_data = list(zip(gps_distances_x, gps_distances_y, gps_distances_z,estimated_positions_x,estimated_positions_y,estimated_positions_z))

# Specify the file name
csv_file = "position_gps_vs_estimated.csv"

# Open the CSV file in write mode
with open(csv_file, mode='w', newline='') as file:
    # Create a CSV writer object
    writer = csv.writer(file)
    
    # Write the header
    writer.writerow(['gps_x', 'gps_y', 'gps_z','fused_x','fused_y','fused_z'])
    
    # Write the data
    writer.writerows(acceleration_data)

print(f"CSV file '{csv_file}' has been created successfully.")




plot_distance(gps_distances_x, estimated_positions_x)
plot_distance(gps_distances_y, estimated_positions_y)
plot_distance(gps_distances_z, estimated_positions_z)

fig6 = plt.figure()
ax6 = fig6.add_subplot(111, projection='3d')
# Plot quiver using roll, pitch, and yaw angles
ax6.plot(estimated_positions_x,estimated_positions_y,estimated_positions_z,'g')
ax6.plot(gps_distances_x,gps_distances_y,gps_distances_z,'r')
# ax6.set_xlim([-10, 10])  # Limit x-axis from 0 to 10
# ax6.set_ylim([-10, 10])  # Limit y-axis from 0 to 10
# ax6.set_zlim([-10, 10])  # Limit z-axis from 0 to 10
ax6.set_xlabel('X')
ax6.set_ylabel('Y')
ax6.set_zlabel('Z')
ax6.set_title('Unit Vector Quiver Plot')
### now computing 
plt.show()
#plot_position_orientation(estimated_positions_x, estimated_positions_y, estimated_positions_z, Read_Data.rollf, Read_Data.pitchf, Read_Data.yawf,const.vector_scale)
plot_cylinder_and_points(const.x_cylinder, const.y_cylinder, const.z_cylinder,const.antradius,const.antheight, x_points=estimated_positions_x, y_points=estimated_positions_y, z_points=estimated_positions_z)
plot_animation(estimated_positions_x, estimated_positions_y, estimated_positions_z, Read_Data.rollf, Read_Data.pitchf, Read_Data.yawf,const.vector_scale)
# plot_animation_orient(estimated_positions_x, estimated_positions_y, estimated_positions_z, Read_Data.rollf, Read_Data.pitchf, Read_Data.yawf,const.vector_scale)

azimuths, elevations = to_azimuth_elevation_with_position_array(Read_Data.rollf, Read_Data.pitchf, Read_Data.yawf, const.observer_position, estimated_positions_x, estimated_positions_y, estimated_positions_z)

# Save to CSV
with open('angles.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for i in range(len(azimuths)):
        writer.writerow([(azimuths[i]),(elevations[i])])

print("Angles saved to angles.csv")