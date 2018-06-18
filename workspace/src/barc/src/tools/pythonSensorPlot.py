"""
    File name: pythonSensorPlot.py
    Author: Shuqi Xu
    Email: shuqixu@berkeley.edu (xushuqi8787@gmail.com)
    Python Version: 2.7.12
"""
import numpy as np
import os
import sys
import pdb
import matplotlib.pyplot as plt

homedir = os.path.expanduser("~")
pathSave = os.path.join(homedir,"barc_debugging/estimator_imu.npz")
npz_imu = np.load(pathSave)
psiDot_his    	= npz_imu["psiDot_his"]
roll_his      	= npz_imu["roll_his"]
pitch_his     	= npz_imu["pitch_his"]
yaw_his      	= npz_imu["yaw_his"]
yaw_raw_his     = npz_imu["yaw_raw_his"]
ax_his      	= npz_imu["ax_his"]
ay_his      	= npz_imu["ay_his"]
imu_time  		= npz_imu["imu_time"]
print "Finish loading data from", pathSave

pathSave = os.path.join(homedir,"barc_debugging/estimator_gps.npz")
npz_gps = np.load(pathSave)
x_his 		= npz_gps["x_his"]
y_his 		= npz_gps["y_his"]
x_ply_his 	= npz_gps["x_ply_his"]
y_ply_his 	= npz_gps["y_ply_his"]
gps_time  	= npz_gps["gps_time"]
print "Finish loading data from", pathSave

pathSave = os.path.join(homedir,"barc_debugging/estimator_enc.npz")
npz_enc = np.load(pathSave)
v_fl_his 	= npz_enc["v_fl_his"]
v_fr_his 	= npz_enc["v_fr_his"]
v_rl_his 	= npz_enc["v_rl_his"]
v_rr_his 	= npz_enc["v_rr_his"]
v_meas_his  = npz_enc["v_meas_his"]
enc_time  	= npz_enc["enc_time"]
print "Finish loading data from", pathSave

pathSave = os.path.join(homedir,"barc_debugging/estimator_ecu.npz")
npz_ecu = np.load(pathSave)
a_his 		= npz_ecu["a_his"]
df_his 		= npz_ecu["df_his"]
ecu_time  	= npz_ecu["ecu_time"]
print "Finish loading data from", pathSave

# FIGURE 2 plotting of IMU data
num_plot = 3
fig = plt.figure("Imu")
ax1 = fig.add_subplot(num_plot,1,1,ylabel="IMU yaw")
ax1.plot(imu_time,yaw_his, label="yaw")
ax1.legend()
ax1.grid()
ax2 = fig.add_subplot(num_plot,1,2,ylabel="IMU acc & psidot")
ax2.plot(imu_time,psiDot_his,label="psiDot")
ax2.plot(imu_time,ax_his,label="ax")
ax2.plot(imu_time,ay_his,label="ay")
ax2.legend()
ax2.grid()
ax3 = fig.add_subplot(num_plot,1,3,ylabel="pitch & roll angle")
ax3.plot(imu_time,roll_his,label="roll angle")
ax3.plot(imu_time,pitch_his,label="pitch angle")
ax3.legend()
ax3.grid()

# raw data and estimation data comparison
num_plot = 1
fig = plt.figure("raw data and est data comparison")
ax4 = fig.add_subplot(num_plot,1,1,ylabel="psidot")
ax4.plot(imu_time, psiDot_his, 	".", label="psidot_meas")
ax4.plot(imu_time, yaw_his, 	".", label="yaw_meas")
ax4.plot(imu_time, yaw_raw_his, ".", label="yaw_raw")
ax4.legend()
ax4.grid()

plt.show()