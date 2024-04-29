import serial
import const
import numpy as np
import filter_data
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import csv
# Open serial port
vx0 = 0
vy0 = 0
vz0 = 0
x0 = 0
y0 = 0
z0 = 0

# Time step

ser = serial.Serial('COM3', 115200)  

QA = 0.01 * np.eye(3)  # Process noise covariance matrix
RA = 0.1 * np.eye(3)  # Measurement noise covariance matrix
# Initialize lists to store roll, pitch, yaw, x, y, z data
roll= []
pitch =[]
yaw=  []
acclerationX = []
acclerationY = []
acclerationZ = []
longitude=[]
latitude=[]
altitude=[]

# Read and plot 30 values
# Read and plot 30 values
for _ in range(const.num_steps):
    # Read data from serial port
    data = ser.readline().decode().strip().split(',')
    if len(data) == 9:  # Assuming the format is roll,pitch,yaw,x,y,z,dx,dy,dz
        try:
            print(data)
            # Convert data to float
            roll_, yaw_, pitch_, ax, ay, az,lon,lat,alt = map(float, data)
            roll.append(roll_)
            pitch.append(pitch_)
            yaw.append(yaw_)
            # Append positional change

            accleration_list=[ax,ay,az]
            r = R.from_euler('xyz', [roll_, yaw_, pitch_],degrees=True)

            rotated_accel = r.apply(accleration_list)
           
            acclerationX.append(rotated_accel[0])
            acclerationY.append(rotated_accel[1])
            acclerationZ.append(rotated_accel[2])
            # acclerationX.append(ax)
            # acclerationY.append(ay)
            # acclerationZ.append(az)
            longitude.append(lon)
            latitude.append(lat)
            altitude.append(alt)
        except ValueError:
            print("Invalid data format")
ser.close()
acclerationX=np.array(acclerationX)
acclerationY=np.array(acclerationY)
acclerationZ=np.array(acclerationZ)
rollf=np.array(roll)
pitchf=np.array(pitch)
yawf=np.array(yaw)

longitude=np.array(longitude)
latitude=np.array(latitude)
altitude=np.array(altitude)
filtered_acc_data=[]
# Process accelerometer data

# Combine the arrays into a list of tuples (ax, ay, az)
acceleration_data = list(zip(rollf, pitchf, yawf))

# Specify the file name
csv_file = "roll_pitch_yaw_data.csv"

# Open the CSV file in write mode
with open(csv_file, mode='w', newline='') as file:
    # Create a CSV writer object
    writer = csv.writer(file)
    
    # Write the header
    writer.writerow(['Roll', 'pitch', 'yaw'])
    
    # Write the data
    writer.writerows(acceleration_data)

print(f"CSV file '{csv_file}' has been created successfully.")

# Plot filtered accelerometer data

axf=filter_data.butter_2nd_order(acclerationX)
ayf=filter_data.butter_2nd_order(acclerationY)
azf=filter_data.butter_2nd_order(acclerationZ)

fig, axes = plt.subplots(3, 1, figsize=(8, 12))
# Plot x distance
axes[0].plot(acclerationX, color='r', label='Original Acceleration')
axes[0].plot(axf, color='b', label='Butterfiltred Acceleration')
axes[0].legend()
axes[0].set_title('Acceleration in X Direction')
# Plot y distance
axes[1].plot(acclerationY, color='r', label='Original Acceleration')
axes[1].plot(ayf, color='b', label='Butter Filtered Acceleration')
axes[1].legend()
axes[1].set_title('Acceleration in Y Direction')
axes[2].plot(acclerationZ, color='r', label='Original Acceleration')
axes[2].plot(azf, color='b', label='Butter Filtered Acceleration')
axes[2].legend()
axes[2].set_title('Acceleration in Z Direction')
##process gps data
fig1, axes1 = plt.subplots(3, 1, figsize=(8, 12))
# Plot x distance
axes1[0].plot(longitude, color='r', label='Original longitude')
axes1[0].legend()
axes1[0].set_title('longitude (E)')
# Plot y distance
axes1[1].plot(latitude, color='r', label='Original latitude')
axes1[1].legend()
axes1[1].set_title('latitude(N)')

axes1[2].plot(altitude, color='r', label='Original altitude')
axes1[2].legend()
axes1[2].set_title('Altitude')
plt.show()
