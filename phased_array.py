import csv
import random

# Define the number of data points
num_points = 10

# Generate random steering angles as double precision floating-point numbers
startA = [random.uniform(0, 130) for _ in range(num_points)]
startE = [random.uniform(0, 89) for _ in range(num_points)]

# Combine angles into a list of tuples
angles = list(zip(startA, startE))

# Save angles to CSV file
with open('angles.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerows(angles)
