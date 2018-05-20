import numpy as np
import os
import matplotlib.pyplot as plt


# FIGURE 1 plotting of estimator output data
homedir = os.path.expanduser("~")
pathSave = os.path.join(homedir,"barc_debugging/estimator_output.npz")
npz_output = np.load(pathSave)
# yaw_true_his 		= npz_output["yaw_true_his"]
# psiDot_true_his 	= npz_output["psiDot_true_his"]
x_est_his     		= npz_output["x_est_his"]
y_est_his     		= npz_output["y_est_his"]
x_est_2_his     	= npz_output["x_est_2_his"]
y_est_2_his      	= npz_output["y_est_2_his"]
estimator_time 		=npz_output["estimator_time"]
psi_drift_est_his 	=npz_output["psi_drift_est_his"]
psi_drift_est_2_his	=npz_output["psi_drift_est_2_his"]
psi_est_his 		=npz_output["psi_est_his"]
psi_est_2_his 		=npz_output["psi_est_2_his"]
vx_est_his          =npz_output["vx_est_his"] 
vy_est_his          =npz_output["vy_est_his"] 
ax_est_his          =npz_output["ax_est_his"] 
ay_est_his          =npz_output["ay_est_his"] 
psi_dot_est_his     =npz_output["psi_dot_est_his"]  
v2_est_his          =npz_output["v2_est_his"] 
vel_meas_his        =npz_output["vel_meas_his"] 
a_his  				=npz_output["a_his"]
df_his  			=npz_output["df_his"]
# df_lp_his 			=npz_output["df_lp_his"]
pathSave = os.path.join(homedir,"barc_debugging/estimator_imu.npz")
npz_imu = np.load(pathSave)
imu_time  		  = npz_imu["imu_time"]
psidot_raw_his    = npz_imu["psidot_raw_his"]
a_x_raw_his       = npz_imu["a_x_raw_his"]
a_y_raw_his       = npz_imu["a_y_raw_his"]
a_x_meas_his      = npz_imu["a_x_meas_his"]
a_y_meas_his      = npz_imu["a_y_meas_his"]
roll_raw_his      = npz_imu["roll_raw_his"]
pitch_raw_his     = npz_imu["pitch_raw_his"]
yaw_raw_his       = npz_imu["yaw_raw_his"]
yaw_his           = npz_imu["yaw_his"]
yaw0_his          = npz_imu["yaw0_his"]
yaw_meas_his      = npz_imu["yaw_meas_his"]
pathSave = os.path.join(homedir,"barc_debugging/estimator_hedge_imu_fusion.npz")
npz_hedge_imu_fusion = np.load(pathSave)
gps_imu_fusion_time = npz_hedge_imu_fusion["gps_imu_fusion_time"]
qx_his = npz_hedge_imu_fusion["qx_his"]
qy_his = npz_hedge_imu_fusion["qy_his"]
qz_his = npz_hedge_imu_fusion["qz_his"]
qw_his = npz_hedge_imu_fusion["qw_his"]
ax_his = npz_hedge_imu_fusion["ax_his"]
ay_his = npz_hedge_imu_fusion["ay_his"]
az_his = npz_hedge_imu_fusion["az_his"]
vx_his = npz_hedge_imu_fusion["vx_his"]
vy_his = npz_hedge_imu_fusion["vy_his"]
vz_his = npz_hedge_imu_fusion["vz_his"]
gps_x_his = npz_hedge_imu_fusion["gps_x_his"]
gps_y_his = npz_hedge_imu_fusion["gps_y_his"]
gps_time  = npz_hedge_imu_fusion["gps_time"]

num_plot = 3
fig = plt.figure("Estimator_output")
ax1 = fig.add_subplot(num_plot,1,1,ylabel="Psi_estimation")
ax1.plot(estimator_time,psi_drift_est_his, 	label="psi_drift_1")
ax1.plot(estimator_time,psi_drift_est_2_his,label="psi_drift_2")
ax1.plot(estimator_time,psi_est_his, 		label="psi_est_1")
ax1.plot(estimator_time,psi_est_2_his,		label="psi_est_2")
ax1.plot(imu_time,yaw_meas_his,				label="yaw_meas")
ax1.plot(estimator_time,np.cumsum(psi_dot_est_his*0.02), label="yaw by integrate")
ax1.legend()
ax1.grid()
ax2 = fig.add_subplot(num_plot,1,2,ylabel="v estimation")
ax2.plot(vx_est_his,label="vx_est")
ax2.plot(v2_est_his,label="v_est_2")
ax2.plot(vel_meas_his,label="v_meas")
ax2.plot(vy_est_his,label="vy_est")
ax2.plot(np.sqrt(vy_est_his**2+vx_est_his**2),label="sqrt")
ax2.legend()
ax2.grid()
ax3 = fig.add_subplot(num_plot,1,3,ylabel="acc & psidot estimation")
ax3.plot(ax_est_his,label="ax")
ax3.plot(ay_est_his,label="ay")
ax3.plot(psi_dot_est_his,label="psidot")
ax3.legend()
ax3.grid()

# FIGURE 2 plotting of IMU data
num_plot = 3
fig = plt.figure("Imu raw data")
ax1 = fig.add_subplot(num_plot,1,1,ylabel="IMU yaw")
ax1.plot(yaw_raw_his, label="yaw_raw")
ax1.plot(yaw_his, label="yaw = unwrap(yaw_raw)")
ax1.plot(yaw0_his, label="yaw0")
ax1.plot(yaw_meas_his, label="yaw_meas = yaw - yaw0")
ax1.legend()
ax1.grid()
ax2 = fig.add_subplot(num_plot,1,2,ylabel="IMU acc & psidot")
ax2.plot(psidot_raw_his,label="psidot_raw")
ax2.plot(a_x_raw_his,label="ax_raw")
ax2.plot(a_y_raw_his,label="ay_raw")
ax2.plot(a_x_meas_his,label="ax_meas")
ax2.plot(a_y_meas_his,label="ay_meas")
ax2.legend()
ax2.grid()
ax3 = fig.add_subplot(num_plot,1,3,ylabel="pitch & roll angle")
ax3.plot(roll_raw_his,label="roll angle")
ax3.plot(pitch_raw_his,label="pitch angle")
ax3.legend()
ax3.grid()

# # GPS comparison
# num_plot = 2
# fig = plt.figure("GPS EST position comparison")
# ax1 = fig.add_subplot(num_plot,1,1,ylabel="x")
# ax1.plot(gps_time, gps_x_his, 			label="gps")
# ax1.plot(estimator_time, x_est_his, 	label="EST 1")
# ax1.plot(estimator_time, x_est_2_his, 	label="EST 2")
# ax1.legend()
# ax1.grid()
# ax2 = fig.add_subplot(num_plot,1,2,ylabel="y")
# ax2.plot(gps_time, gps_y_his, 			label="gps")
# ax2.plot(estimator_time, y_est_his, 	label="EST 1")
# ax2.plot(estimator_time, y_est_2_his, 	label="EST 2")
# ax2.legend()
# ax2.grid()

# raw data and estimation data comparison
num_plot = 3
fig = plt.figure("raw data and est data comparison")
ax2 = fig.add_subplot(num_plot,1,1,ylabel="ax")
ax2.plot(imu_time, a_x_meas_his, ".", 	label="ax_meas")
ax2.plot(estimator_time, ax_est_his, 	label="ax_est")
ax2.plot(estimator_time, a_his, "--",	label="cmd.acc")
ax2.legend()
ax2.grid()
ax3 = fig.add_subplot(num_plot,1,2,ylabel="ay")
ax3.plot(imu_time, a_y_meas_his, ".", 	label="ay_meas")
ax3.plot(estimator_time, ay_est_his, 	label="ay_est")
ax3.plot(estimator_time, df_his, "--",	label="cmd.df")
ax3.legend()
ax3.grid()
ax4 = fig.add_subplot(num_plot,1,3,ylabel="psidot")
ax4.plot(imu_time, psidot_raw_his, ".", label="psidot_meas")
ax4.plot(estimator_time,psi_dot_est_his,label="psidot_est")
ax4.plot(estimator_time, df_his, "--",	label="cmd.df")
ax4.legend()
ax4.grid()

fig = plt.figure("input")
ax4 = fig.add_subplot(1,1,1,ylabel="ax")
ax4.plot(estimator_time, df_his, "-",	label="cmd.df")
ax4.plot(estimator_time, a_his, "--",	label="cmd.a")
ax4.legend()
ax4.grid()

fig = plt.figure("yaw compare")
ax4 = fig.add_subplot(1,1,1,ylabel="yaw")
# ax4.plot(gps_time, yaw_true_his[:-1], "--",	label="yaw true")
ax4.plot(estimator_time, psi_est_his, "--",	label="yaw est 1")
ax4.plot(estimator_time, psi_est_2_his, "--",	label="yaw est 2")
ax4.legend()
ax4.grid()

""" GPS imu fusion data plot
# Figure 3 hedge_imu_fusion_data
num_plot = 4
fig = plt.figure("hedge imu fusion data")
ax1 = fig.add_subplot(num_plot,1,1,ylabel="q data, which needs to compare with IMU/data")
ax1.plot(qx_his,	label="qx")
ax1.plot(qy_his,	label="qy")
ax1.plot(qz_his,	label="qz")
ax1.plot(qw_his,	label="qw")
ax1.legend()
ax1.grid()
ax2 = fig.add_subplot(num_plot,1,2,ylabel="acc")
ax2.plot(ax_his,label="ax")
ax2.plot(ay_his,label="ay")
ax2.plot(az_his,label="az")
ax2.legend()
ax2.grid()
ax3 = fig.add_subplot(num_plot,1,3,ylabel="v")
ax3.plot(vx_his,label="vx")
ax3.plot(vy_his,label="vy")
ax3.plot(vz_his,label="vz")
ax3.legend()
ax3.grid()
ax4 = fig.add_subplot(num_plot,1,4,ylabel="gps position")
ax4.plot(gps_x_his,label="x")
ax4.plot(gps_y_his,label="y")
ax4.legend()
ax4.grid()
"""

""" GPS IMU raw data plot
# Figure 4 hedge_imu_raw_data
pathSave = os.path.join(homedir,"barc_debugging/estimator_hedge_imu_raw.npz")
npz_hedge_imu_raw = np.load(pathSave)
acc_x_his = npz_hedge_imu_raw["acc_x_his"]
acc_y_his = npz_hedge_imu_raw["acc_y_his"]
acc_z_his = npz_hedge_imu_raw["acc_z_his"]
gyro_x_his = npz_hedge_imu_raw["gyro_x_his"]
gyro_y_his = npz_hedge_imu_raw["gyro_y_his"]
gyro_z_his = npz_hedge_imu_raw["gyro_z_his"]
compass_x_his = npz_hedge_imu_raw["compass_x_his"]
compass_y_his = npz_hedge_imu_raw["compass_y_his"]
compass_z_his = npz_hedge_imu_raw["compass_z_his"]

num_plot = 3
fig = plt.figure("hedge imu raw data")
ax1 = fig.add_subplot(num_plot,1,1,ylabel="gyro")
ax1.plot(gyro_x_his,	label="gyro_x_his")
ax1.plot(gyro_y_his,	label="gyro_y_his")
ax1.plot(gyro_z_his,	label="gyro_z_his")
ax1.legend()
ax1.grid()
ax2 = fig.add_subplot(num_plot,1,2,ylabel="acc")
ax2.plot(acc_x_his,label="acc_x_his")
ax2.plot(acc_y_his,label="acc_y_his")
ax2.plot(acc_z_his,label="acc_z_his")
ax2.legend()
ax2.grid()
ax3 = fig.add_subplot(num_plot,1,3,ylabel="compass")
ax3.plot(compass_x_his,label="compass_x")
ax3.plot(compass_y_his,label="compass_y")
ax3.plot(compass_z_his,label="compass_z")
ax3.legend()
ax3.grid()
"""

plt.show()