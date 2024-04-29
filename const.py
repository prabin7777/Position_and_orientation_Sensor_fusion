import numpy as np
cutoff_frequency = 5  #     Cut off frequency for acclerometer data butterworth
sampling_rate = 100   # Sampling rate foer the data 
dt=0.01   ## delta time betn samples
num_steps=200 # sample size 
vector_scale=15 #quiver plot ko vectoe scale
x_cylinder=500
y_cylinder=500
z_cylinder=50
antradius=2
antheight=15
observer_position=np.array([x_cylinder,y_cylinder,z_cylinder])