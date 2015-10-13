# -*- coding: utf-8 -*-
"""
Sensor model test file
Created on Mon Sep 28 2015

@author: jgon13
"""

from IMU_sensor_model import estimate_position, signal, compute_yaw_rate
from numpy import  pi, zeros
import pandas as pd
import Tkinter, tkFileDialog
import matplotlib.pyplot as plt

#%% Load raw measurements
# Read data from csv file
prompt = Tkinter.Tk()
prompt.attributes("-topmost",True)
prompt.withdraw()

options = {}
options['filetypes'] = [('all files', '.*')]
options['initialdir'] = "C:\Users\ych09_000\Desktop\IMU_and_PID"
data_file = tkFileDialog.askopenfilename(**options)

IMU_data = pd.read_csv(data_file)

## raw measurements
t       = IMU_data['t_s'].values
a_x_RAW = IMU_data['a_x'].values 
a_y_RAW = IMU_data['a_y'].values 
psi_RAW = IMU_data['yaw'].values 


#%% Initialize system
# storage variables
N       = t.size
v_x     = zeros(N)
v_y     = zeros(N)
X       = zeros(N)
Y       = zeros(N)
v_x_mv_avg = zeros(N)
v_y_mv_avg = zeros(N)

# Initialization
g           = 9.81
t0          = 0

# filter parameters
# aph   := smoothing factor, (all filtered)   0 <=   aph   <= 1  (no filter)
# n     := size of moving average block
a1          = 0.005
a2          = 0.05
n           = 200

# measurement, body frame, and global frame data data
Y_data      = signal(y0 = [0,0,0,0], a = [a1,a1,1,a2], method = 'lp')
BF_data     = signal(y0 = [0,0], n = n, method = 'mvg')
GF_data     = signal(y0 = [0,0,0,0], method = None)
X_estimate  = (BF_data, GF_data)

#%% Perform simulation
for i in range(N-2):
    # get new measurements (remember negative sign !)
    a_x_new     = a_x_RAW[i] * g 
    a_y_new     = -a_y_RAW[i] * g
    psi_new     = -psi_RAW[i] * (pi/180.0)
    tf          = t[i]
    dt          = (tf - t0)
    
    psi_prev    = Y_data.getSignal('raw')[2]
    w_z_new     = compute_yaw_rate(psi_prev, psi_new, dt)
    
    # filter signals
    Y_data.update( [a_x_new, a_y_new, psi_new, w_z_new] )
    
    # estimate position
    estimate_position(Y_data, X_estimate, dt)  
    
    # update terms
    t0  = tf
    
    # record velocities
    (v_x[i], v_y[i])    = BF_data.getSignal()
    (X[i], Y[i], _, _)  = GF_data.getSignal()


#%%   Plot results
font = {'family' : 'serif', 
        'color'  : 'black',
        'weight' : 'normal',
        'size'   : 12}
k0  = 0
kf  = N-2
plt.figure(figsize=(15,8))

plt.subplot(121)
p1, = plt.plot(X[k0:kf],Y[k0:kf],'k-')
plt.plot(X[k0],Y[k0],'gs')
plt.plot(X[kf],Y[kf],'rs')
plt.grid(axis=u'both')
plt.gca().set_aspect('equal', adjustable='box')
plt.xlabel(r'$X$ [m]', fontdict=font)
plt.ylabel(r'$Y$ [m]', fontdict=font)
plt.title(r'Global position of RC vehicle', fontdict=font)

plt.subplot(122)
v1, = plt.plot(t[k0:kf],v_x[k0:kf],'r-')
v2, = plt.plot(t[k0:kf],v_y[k0:kf],'b-')
plt.xlabel(r'$t$ [s]', fontdict=font)
plt.grid(axis=u'both')
plt.ylabel(r'velocity [m/s]', fontdict=font)
plt.title(r'body frame velocities', fontdict=font)
plt.legend((v1,v2),(r'$v_x$', r'$v_y$'))

plt.suptitle(r'Low Pass Filter values of ($a_x, a_y, \omega_z$) = (%.3f, %.3f, %.3f)' % (a1, a1, a2))