import numpy as np
import os
import sys
import pdb
import matplotlib.pyplot as plt
homedir = os.path.expanduser("~")

# FIGURE 1 plotting of estimator output data
homedir = os.path.expanduser("~")
pathSave = os.path.join(homedir,"barc_debugging_play_back/estimator_output.npz")
npz_output = np.load(pathSave)
x_est_his     		= npz_output["x_est_his"]
y_est_his     		= npz_output["y_est_his"]
vx_est_his          =npz_output["vx_est_his"] 
vy_est_his          =npz_output["vy_est_his"] 
ax_est_his          =npz_output["ax_est_his"] 
ay_est_his          =npz_output["ay_est_his"] 
psiDot_est_his     	=npz_output["psiDot_est_his"]  
yaw_est_his     	=npz_output["yaw_est_his"]  
estimator_time 		=npz_output["estimator_time"]

pathSave = os.path.join(homedir,"barc_debugging_play_back/estimator_imu.npz")
npz_imu = np.load(pathSave)
psiDot_his    	= npz_imu["psiDot_his"]
roll_his      	= npz_imu["roll_his"]
pitch_his     	= npz_imu["pitch_his"]
yaw_his      	= npz_imu["yaw_his"]
ax_his      	= npz_imu["ax_his"]
ay_his      	= npz_imu["ay_his"]
imu_time  		= npz_imu["imu_time"]
imu_time  		= estimator_time

pathSave = os.path.join(homedir,"barc_debugging_play_back/estimator_gps.npz")
npz_gps = np.load(pathSave)
x_his 		= npz_gps["x_his"]
y_his 		= npz_gps["y_his"]
gps_time  	= npz_gps["gps_time"]
gps_time  	= estimator_time

pathSave = os.path.join(homedir,"barc_debugging_play_back/estimator_enc.npz")
npz_enc = np.load(pathSave)
v_fl_his 	= npz_enc["v_fl_his"]
v_fr_his 	= npz_enc["v_fr_his"]
v_rl_his 	= npz_enc["v_rl_his"]
v_rr_his 	= npz_enc["v_rr_his"]
enc_time  	= npz_enc["enc_time"]
enc_time  	= estimator_time

pathSave = os.path.join(homedir,"barc_debugging_play_back/estimator_ecu.npz")
npz_ecu = np.load(pathSave)
a_his 		= npz_ecu["a_his"]
df_his 		= npz_ecu["df_his"]
ecu_time  	= npz_ecu["ecu_time"]
ecu_time  	= estimator_time
"""
# pdb.set_trace()
# FIGURE 1 plotting of estimator data
num_col_plt = 3
num_row_plt = 1
fig = plt.figure("Estimator")
ax1 = fig.add_subplot(num_col_plt,num_row_plt,1,ylabel="yaw_estimation")
ax1.plot(estimator_time,yaw_est_his,label="yaw_est")
ax1.legend()
ax1.grid()
ax2 = fig.add_subplot(num_col_plt,num_row_plt,2,ylabel="v estimation")
ax2.plot(estimator_time,vx_est_his,label="vx_est")
ax2.plot(estimator_time,vy_est_his,label="vy_est")
ax2.legend()
ax2.grid()
ax3 = fig.add_subplot(num_col_plt,num_row_plt,3,ylabel="acc & psidot estimation")
ax3.plot(estimator_time,ax_est_his,label="ax")
ax3.plot(estimator_time,ay_est_his,label="ay")
ax3.plot(estimator_time,psiDot_est_his,label="psiDot")
ax3.legend()
ax3.grid()
# FIGURE 2 plotting of IMU data
num_plot = 3
fig = plt.figure("Imu")
ax1 = fig.add_subplot(num_plot,1,1,ylabel="IMU yaw")
# ax1.plot(imu_time,yaw_his, label="yaw")
ax1.legend()
ax1.grid()
ax2 = fig.add_subplot(num_plot,1,2,ylabel="IMU acc & psidot")
ax2.plot(imu_time,psiDot_his,label="psiDot")
ax2.plot(imu_time,ax_his,label="ax")
ax2.plot(imu_time,ay_his,label="ay")
print np.mean(ax_his), np.mean(ay_his)
ax2.legend()
ax2.grid()
ax3 = fig.add_subplot(num_plot,1,3,ylabel="pitch & roll angle")
# ax3.plot(imu_time,roll_his,label="roll angle")
# ax3.plot(imu_time,pitch_his,label="pitch angle")
ax3.legend()
ax3.grid()
# GPS comparison
num_plot = 2
fig = plt.figure("GPS")
ax1 = fig.add_subplot(num_plot,1,1,ylabel="x")
ax1.plot(gps_time, x_his, 	label="x")
ax1.plot(estimator_time, x_est_his, 	label="x_est")
ax1.legend()
ax1.grid()
ax2 = fig.add_subplot(num_plot,1,2,ylabel="y")
ax2.plot(gps_time, y_his, 	label="y")
ax2.plot(estimator_time, y_est_his, 	label="y_est")
ax2.legend()
ax2.grid()
# ecu plot
fig = plt.figure("input")
ax4 = fig.add_subplot(1,1,1,ylabel="ax")
ax4.plot(ecu_time, df_his, "-",	label="cmd.df")
ax4.plot(ecu_time, a_his, "--",	label="cmd.a")
ax4.legend()
ax4.grid()
# enc plot
fig = plt.figure("encoder")
ax4 = fig.add_subplot(1,1,1,ylabel="ax")
ax4.plot(enc_time, v_fl_his, "--",	label="fl")
ax4.plot(enc_time, v_fr_his, "--",	label="fr")
ax4.plot(enc_time, v_rl_his, "-",	label="rl")
ax4.plot(enc_time, v_rr_his, "-",	label="rr")
ax4.legend()
ax4.grid()
"""
# trajectory
fig = plt.figure("track x-y plot")
ax1 = fig.add_subplot(1,1,1,ylabel="track x-y plot")
# ax1.plot(l.nodes[0],l.nodes[1],color="grey",linestyle="--", alpha=0.3)
# ax1.plot(l.nodes_bound1[0],l.nodes_bound1[1],color="red",alpha=0.3)
# ax1.plot(l.nodes_bound2[0],l.nodes_bound2[1],color="red",alpha=0.3)
ax1.axis("equal")
ax1.plot(x_est_his,y_est_his,color="green")
ax1.plot(x_his,y_his,"-or")
ax1.legend()
"""
# pdb.set_trace()
# raw data and estimation data comparison
num_plot = 3
fig = plt.figure("raw data and est data comparison")
ax2 = fig.add_subplot(num_plot,1,1,ylabel="ax")
ax2.plot(imu_time, ax_his, ".", 	label="ax_meas")
ax2.plot(estimator_time, ax_est_his, 	label="ax_est")
# ax2.plot(estimator_time, a_his, "--",	label="cmd.acc")
ax2.legend()
ax2.grid()
ax3 = fig.add_subplot(num_plot,1,2,ylabel="ay")
ax3.plot(imu_time, ay_his, ".", 	label="ay_meas")
ax3.plot(estimator_time, ay_est_his, 	label="ay_est")
# ax3.plot(estimator_time, df_his, "--",	label="cmd.df")
ax3.legend()
ax3.grid()
ax4 = fig.add_subplot(num_plot,1,3,ylabel="psidot")
ax4.plot(imu_time, psiDot_his, ".", label="psidot_meas")
ax4.plot(estimator_time,psiDot_est_his,label="psidot_est")
# ax4.plot(estimator_time, df_his, "--",	label="cmd.df")
ax4.legend()
ax4.grid()
"""

plt.show()