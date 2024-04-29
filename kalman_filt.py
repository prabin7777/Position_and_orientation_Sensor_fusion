from filterpy.kalman import KalmanFilter
import numpy as np
import Read_Data
import matplotlib.pyplot as plt
def estimate_position(acc_x, gps_distances, dt):
    kf = KalmanFilter(dim_x=2, dim_z=1)

    # State transition matrix
    kf.F = np.array([[1, dt],
                     [0, 1]])
   
    # Measurement function matrix
    kf.H = np.array([[1, 0]])
    
    # Process noise covariance matrix yesle gps ko data samasya hunxa noise 
    kf.Q *= 0.0001

    # Measurement noise covariance matrix 
    kf.R *= 0.1
    
    # Control input matrix (acceleration)
    kf.B = np.array([[0.5*dt**2],
                     [dt]])

    # Initial state estimate
    kf.x = np.array([[0],   # initial position
                     [0]])  # initial velocity
    
    # Initial state covariance
    kf.P *= 1000

    estimated_positions = []
    for i, (a_x, gps_dist) in enumerate(zip(acc_x, gps_distances)):
        # Predict
        kf.predict(u=a_x)
        # Update using GPS distance measurement
        kf.update(gps_dist)
        estimated_positions.append(kf.x[0, 0])
    return estimated_positions
    
    
    

    

def plot_distance(gps_distances, estimated_positions):
    plt.plot(range(len(gps_distances)), gps_distances, label='GPS Distance')
    plt.plot(range(len(estimated_positions)),(estimated_positions), label='Estimated Distance')
    plt.xlabel('Time Step')
    plt.ylabel('Distance')
    plt.title('GPS Distance vs IMU Fused distance')
    plt.legend()
    plt.grid(True)
    plt.show()


