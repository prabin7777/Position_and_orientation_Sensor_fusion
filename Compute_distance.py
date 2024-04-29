import numpy as np
import matplotlib.pyplot as plt

import const
# Example accelerometer data

# Lists to store estimated positions
estimated_positions = []

def compute_distance_from_accleration(axf, ayf, azf):
    # Integrate acceleration to get velocity
    # vgx=np.array(vgx)
    # vgy=np.array(vgy)
    # vgz=np.array(vg
    vx=np.zeros(const.num_steps)
    vy=np.zeros(const.num_steps)
    vz=np.zeros(const.num_steps)
    x_d=np.zeros(const.num_steps)
    y_d=np.zeros(const.num_steps)
    z_d=np.zeros(const.num_steps)
    dt=const.dt
    for i in range(1,len(axf)):
        vx[i]=vx[i-1]+axf[i]*dt
        vy[i]=vx[i-1]+ayf[i]*dt
        vz[i]=vx[i-1]+azf[i]*dt

        x_d[i]=x_d[i-1]+vx[i-1]*dt+0.5*axf[i]*dt*dt
        y_d[i]=y_d[i-1]+vy[i-1]*dt+0.5*ayf[i]*dt*dt
        z_d[i]=z_d[i-1]+vz[i-1]*dt+0.5*azf[i]*dt*dt


   
    return x_d,y_d,z_d
