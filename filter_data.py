
import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
import const
# Lists to store filtered accelerometer data


# Kalman filter function linear.......... for accleration and gps data noise reduction.
def butter_2nd_order(acc_data):
    # Define filter parameters
    

# Design the filter
    nyquist_frequency = 0.5 * const.sampling_rate
    normalized_cutoff_frequency = const.cutoff_frequency / nyquist_frequency
    b, a = butter(2, normalized_cutoff_frequency, btype='low', analog=False)

# Apply the filter to the data
    filtered_data = filtfilt(b, a, acc_data)


    
    return filtered_data





