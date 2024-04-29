import numpy as np
import matplotlib.pyplot as plt

class ExtendedKalmanFilter:
    def __init__(self, initial_state, initial_covariance, process_noise_cov, measurement_noise_cov):
        self.state = initial_state
        self.covariance = initial_covariance
        self.process_noise_cov = process_noise_cov
        self.measurement_noise_cov = measurement_noise_cov
        self.history = [initial_state]  # Store history of estimated states
    
    def predict(self, motion_model, control_input):
        # Predict next state
        self.state = motion_model(self.state, control_input)
        # Compute Jacobian of the motion model
        F = self.compute_motion_jacobian(motion_model, control_input)
        # Update covariance matrix
        self.covariance = np.dot(F, np.dot(self.covariance, F.T)) + self.process_noise_cov
    
    def update(self, measurement, measurement_model):
        # Compute Jacobian of the measurement model
        H = self.compute_measurement_jacobian(measurement_model)
        # Compute Kalman gain
        K = np.dot(self.covariance, np.dot(H.T, np.linalg.inv(np.dot(H, np.dot(self.covariance, H.T)) + self.measurement_noise_cov)))
        # Update state estimate
        self.state = self.state + np.dot(K, (measurement - measurement_model(self.state)))
        # Update covariance matrix
        self.covariance = np.dot((np.eye(len(self.state)) - np.dot(K, H)), self.covariance)
        # Store estimated state in history
        self.history.append(self.state)
    
    def compute_motion_jacobian(self, motion_model, control_input):
        # Numerically compute the Jacobian of the motion model
        epsilon = 1e-6
        F = np.zeros((len(self.state), len(self.state)))
        for i in range(len(self.state)):
            delta = np.zeros(len(self.state))
            delta[i] = epsilon
            F[:, i] = (motion_model(self.state + delta, control_input) - motion_model(self.state - delta, control_input)) / (2 * epsilon)
        return F
    
    def compute_measurement_jacobian(self, measurement_model):
        # Numerically compute the Jacobian of the measurement model
        epsilon = 1e-6
        H = np.zeros((2, len(self.state))) # Assuming measurement is 2D (x, y)
        for i in range(len(self.state)):
            delta = np.zeros(len(self.state))
            delta[i] = epsilon
            H[:, i] = (measurement_model(self.state + delta) - measurement_model(self.state - delta)) / (2 * epsilon)
        return H

# Example usage:
initial_state = np.array([0, 0])  # Initial position
initial_covariance = np.eye(2) * 0.1  # Initial covariance
process_noise_cov = np.eye(2) * 0.01  # Process noise covariance
measurement_noise_cov = np.eye(2) * 0.1  # Measurement noise covariance

def motion_model(state, control_input):
    # Simple motion model: constant velocity
    dt = 1.0  # Time step
    return state + dt * control_input

def measurement_model(state):
    return state

# Create EKF instance
ekf = ExtendedKalmanFilter(initial_state, initial_covariance, process_noise_cov, measurement_noise_cov)

# Example motion and measurement inputs
control_input = np.array([5, 1]) # Constant velocity
measurement = np.array([2, 3])  # Simulated measurement

# Prediction step
ekf.predict(motion_model, control_input)

# Update step
ekf.update(measurement, measurement_model)

# Plotting
history = np.array(ekf.history)
plt.plot(history[:, 0], history[:, 1], label='Estimated Position', marker='o')
plt.scatter(measurement[0], measurement[1], color='red', label='Measurement')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Extended Kalman Filter 2D Position Estimation')
plt.legend()
plt.grid(True)
plt.show()
