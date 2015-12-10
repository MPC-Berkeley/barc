# -*- coding: utf-8 -*-
"""
Sensor model test file
Created on Mon Sep 28 2015

@author: jgon13
"""

from filtering import  filteredSignal
from state_estimation import estimateVelocity, estimatePosition, estimateAngularAcceleration
from numpy import  pi, zeros, arctan, cos
import pandas as pd
import matplotlib.pyplot as plt
from numpy import array
from numpy import linalg as LA

data_file ='data/2015-12-02_02-34-11.csv'
IMU_data  = pd.read_csv(data_file)

## raw measurements
t          = IMU_data['t_s'].values
ax_IMU_RAW = IMU_data['a_x'].values
ay_IMU_RAW = IMU_data['a_y'].values
psi_RAW    = IMU_data['yaw'].values
w_z_RAW    = IMU_data['w_z'].values
vx_enc     = IMU_data['vx_enc'].values
d_F        = IMU_data['d_F'].values


#%% Initialize system
# filtered state variables
N         = t.size
vx_CoG    = zeros(N)
vy_CoG    = zeros(N)
ay_CoG    = zeros(N)
ax_IMU    = zeros(N)
ay_IMU    = zeros(N)
w_z       = zeros(N)
dot_w_z   = zeros(N)
X         = zeros(N)
Y         = zeros(N)

# set time to start from zero
k0  = 1
kf  = N-2
t0  = t[k0-1]
t   = t - t0

################################
# slip angles
################################
beta  = zeros(N)
alfaF = zeros(N)
alfaR = zeros(N)

################################
# Lateral forces
################################
FyF = zeros(N)
FyR = zeros(N)

#  Vehicle parameters
g       = 9.81
m       = 1.76
Iz 	    = 0.248       # mass of vehicle [kg]
a       = 0.14          # distance from CoG to front axel [m]
b       = 0.14          # distance from CoG to rear axel [m]
L       = a + b         # total length of car
mdl     = (m,Iz,a,b)

# filter parameters
# aph   := smoothing factor, (all filtered)   0 <=   aph   <= 1  (no filter)
# n     := size of moving average block
p_ax        = 0.3         # filter parameter for ax_IMU
p_ay        = p_ax        # filter parameter for ay_IMU
p_psi       = 1
p_wz        = 0.8         # filter parameter for w_z
n           = 25          # moving average filter parameter for v_x, v_y

# measurement signal
# velocity data in the  frame
# position/velocuty data in the global frame
imu_data   	= filteredSignal(y0 = [0,0,0,0], a = [p_ax, p_ay, p_psi, p_wz], method = 'lp')
v_BF     	= filteredSignal(y0 = [0,0], n = n, method = 'mvg')
X_GF     	= filteredSignal(y0 = [0,0,0,0], method = None)

#%% Perform simulation
dwz_k        = 0
wz_km1 = 0
for i in range(k0,kf):
    # get new measurements (remember negative sign !)
    ax_IMU_k     = ax_IMU_RAW[i] * g
    ay_IMU_k     = -ay_IMU_RAW[i] * g
    psi_k     	= -psi_RAW[i] * (pi/180.0)
    wz_k     	= -w_z_RAW[i] * (pi/180.0)
    vx_ENC_k     = vx_enc[i]               # vx_ENC_kp1   = vx_enc[i+1]
    u_k          = -d_F[i]*(pi/180)    # this should not have negative sign in general, mis-recorded sign of steering angle
    
    tf          	= t[i]
    dt          	= (tf - t0)

    # filter signals of IMU  ( in IMU local frame )
    imu_data.update( [ax_IMU_k, ay_IMU_k, psi_k, wz_k] )
    
    # estimate angular acceleration
    dwz_k = estimateAngularAcceleration(imu_data, w_z[i-1], i, dt)

    # estimate velocity in body frame
    # estimate position in global frame
    estimateVelocity(imu_data, v_BF, vx_ENC_k, dwz_k, u_k, mdl, dt)
    estimatePosition(v_BF, X_GF, psi_k, vx_ENC_k, dt)

    # update terms
    t0  = tf

    # record velocities
    (ax_IMU[i], ay_IMU[i], _, _)    = imu_data.getFilteredSignal()
    (vx_CoG[i], vy_CoG[i])          = v_BF.getFilteredSignal()
    (X[i], Y[i], _, _)              = X_GF.getFilteredSignal()
    w_z[i]                          = wz_k
    dot_w_z[i]                      = dwz_k

    # compute side slip
    (vx_k, vy_k)    = v_BF.getFilteredSignal()
    if vx_k >= 0.02:
        beta[i]  = arctan(vy_k /vx_k)*180/pi
        alfaF[i] = arctan((vy_k + a*wz_k)/ vx_k) - u_k
        alfaR[i] = arctan((vy_k - b*wz_k)/ vx_k)
    dot_vy = (vy_CoG[i] - vy_CoG[i-1])/dt
    ay_CoG[i] = dot_vy + vx_k * wz_k

    # compute tire forces
    # m( ayCoG)== Fy,F cos δ + Fy,R
    # Iz*dot(ω ̇z) = aFy,F cos δ − bFy,R
    A = array([[cos(u_k),  1],  [a*cos(u_k), -b]])
    y = array([m*(dot_vy + vx_k * wz_k) , Iz*dot_w_z[i]])
    x = LA.solve(A, y)
    FyF[i] = x[0]
    FyR[i] = x[1]

#%%   Plot results
plt.close('all')
font = {'family' : 'serif',
        'color'  : 'black',
        'weight' : 'normal',
        'size'   : 12}
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
v1, = plt.plot(t[k0:kf], vx_CoG[k0:kf],'r-')
v2, = plt.plot(t[k0:kf], vy_CoG[k0:kf],'b-')
plt.grid(axis=u'both')
plt.xlabel(r'$t$ [s]', fontdict=font)
plt.ylabel(r'velocity [m/s]', fontdict=font)
plt.title(r'body frame velocities', fontdict=font)
plt.legend((v1,v2),(r'$v_x$', r'$v_y$'))
plt.suptitle(r'Low Pass Filter values of ($a_x, a_y, \omega_z$) = (%.3f, %.3f, %.3f)' % (p_ax, p_ay, p_wz))

plt.figure()
plt.plot(t[k0:kf], psi_RAW[k0:kf],'b-', label=r'$\psi$')
plt.plot(t[k0:kf], w_z_RAW[k0:kf],'r-', label=r'raw $\dot{\psi}$')
plt.plot(t[k0:kf], w_z[k0:kf]*180/pi,'g-', label=r'filtered $\dot{\psi}$')
plt.xlabel(r'$t$ [s]', fontdict=font)
plt.title(r'yaw angle and yaw rate', fontdict=font)
plt.legend()

plt.figure()
plt.plot(t[k0:kf], ax_IMU_RAW[k0:kf]*g,'k-', label='raw')
plt.plot(t[k0:kf], ax_IMU[k0:kf],'r-', label='filtered')
plt.xlabel(r'$t$ [s]', fontdict=font)
plt.ylabel(r'$a_x$', fontdict=font)
plt.title(r'longitudinal acceleration $a_x$', fontdict=font)
plt.legend()

plt.figure()
plt.plot(t[k0:kf], -ay_IMU_RAW[k0:kf]*g,'k-',label='IMU-raw')
plt.plot(t[k0:kf], ay_IMU[k0:kf],'r-', label='IMU-filtered')
plt.plot(t[k0:kf], ay_CoG[k0:kf],'r-', label='CoG')
plt.xlabel(r'$t$ [s]', fontdict=font)
plt.ylabel(r'$a_y$', fontdict=font)
plt.title(r'lateral acceleration $a_y$', fontdict=font)
plt.legend()

plt.figure()
plt.plot(t[k0:kf], beta[k0:kf],'g', label=r'$\beta$')
plt.plot(t[k0:kf], alfaF[k0:kf],'b', label=r'$\alpha_F$')
plt.plot(t[k0:kf], alfaR[k0:kf],'r', label=r'$\alpha_R$')
plt.xlabel(r'$t$ [s]', fontdict=font)
plt.title(r'slip angles', fontdict=font)
plt.legend()

plt.figure()
plt.plot(alfaF[k0:kf],FyF[k0:kf],'b*')
plt.xlabel(r'$\alpha_F$ [deg]', fontdict=font)
plt.ylabel(r'$F_{yF}$ [N]', fontdict=font)
plt.title(r'Front lateral force vs front slip angle', fontdict=font)

plt.figure()
plt.plot(alfaR[k0:kf],FyR[k0:kf],'r*')
plt.xlabel(r'$\alpha_R$ [deg]', fontdict=font)
plt.ylabel(r'$F_{yR}$ [N]', fontdict=font)
plt.title(r'Rear lateral force vs rear slip angle', fontdict=font)
