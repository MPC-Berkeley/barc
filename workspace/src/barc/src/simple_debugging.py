import numpy as np
import os
import matplotlib.pyplot as plt
import pdb
from scipy.signal import savgol_filter
import copy

PLOT = False

l_f = 0.125
l_r = 0.125

homedir = os.path.expanduser("~")

"""
# FIGURE 1 plotting of estimator output data

pathSave = os.path.join(homedir,"barc_debugging/estimator_output.npz")
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
"""

steering_str = homedir + "/barc_debugging/steering/"
acc_str = homedir + "/barc_debugging/acceleration/"
steering_dir = os.listdir(steering_str)
acc_dir = os.listdir(acc_str)

X = 0.
d_f = 0.
pwm_entries = 0.

first = True

for file in steering_dir:
	pathSave = steering_str + file + "/estimator_imu.npz"
	npz_imu = np.load(pathSave)
	psiDot_his    	= npz_imu["psiDot_his"]
	roll_his      	= npz_imu["roll_his"]
	pitch_his     	= npz_imu["pitch_his"]
	yaw_his      	= npz_imu["yaw_his"]
	ax_his      	= npz_imu["ax_his"]
	ay_his      	= npz_imu["ay_his"]
	imu_time  		= npz_imu["imu_time"]

	"""
	pathSave = os.path.join(homedir,"barc_debugging/estimator_gps.npz")
	npz_gps = np.load(pathSave)
	x_his 		= npz_gps["x_his"]
	y_his 		= npz_gps["y_his"]
	gps_time  	= npz_gps["gps_time"]
	"""

	pathSave = steering_str + file + "/estimator_enc.npz"
	npz_enc = np.load(pathSave)
	v_fl_his 	= npz_enc["v_fl_his"]
	v_fr_his 	= npz_enc["v_fr_his"]
	v_rl_his 	= npz_enc["v_rl_his"]
	v_rr_his 	= npz_enc["v_rr_his"]
	v_meas_his 	= npz_enc["v_meas_his"]
	enc_time  	= npz_enc["enc_time"]

	"""
	pathSave = os.path.join(homedir,"barc_debugging/estimator_ecu.npz")
	npz_ecu = np.load(pathSave)
	a_his 		= npz_ecu["a_his"]
	df_his 		= npz_ecu["df_his"]
	ecu_time  	= npz_ecu["ecu_time"]
	"""

	# Calculation of steering angle
	steering_pwm = int(file.split("_")[0])
	psi_dot = psiDot_his[150:-25]
	v_x = v_rr_his[150:-25]

	d_f_ = np.arctan(psi_dot * (l_f + l_r) / v_x)

	pwm_entries_ = steering_pwm * np.ones_like(d_f_)
	X_ = np.zeros((len(pwm_entries_), 2))
	X_[:, 0] = pwm_entries_
	X_[:, 1] = np.ones_like(d_f_)

	if first:
		X = X_
		d_f = d_f_
		pwm_entries = pwm_entries_
		first = False
	else:
		X = np.vstack((X, X_))
		d_f = np.hstack((d_f, d_f_))
		pwm_entries = np.hstack((pwm_entries, pwm_entries_))

	if PLOT:
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
		print np.mean(ax_his), np.mean(ay_his)
		ax2.legend()
		ax2.grid()
		ax3 = fig.add_subplot(num_plot,1,3,ylabel="pitch & roll angle")
		ax3.plot(imu_time,roll_his,label="roll angle")
		ax3.plot(imu_time,pitch_his,label="pitch angle")
		ax3.legend()
		ax3.grid()

		# enc plot
		fig = plt.figure("encoder")
		ax4 = fig.add_subplot(1,1,1,ylabel="ax")
		ax4.plot(enc_time, v_fl_his, "--",	label="fl")
		ax4.plot(enc_time, v_fr_his, "--",	label="fr")
		ax4.plot(enc_time, v_rl_his, "-",	label="rl")
		ax4.plot(enc_time, v_rr_his, "-",	label="rr")
		ax4.plot(enc_time, v_meas_his, "-",	label="meas")
		ax4.legend()
		ax4.grid()

		plt.show()

theta = np.matmul(np.matmul(np.linalg.inv(np.matmul(np.transpose(X), X)), np.transpose(X)), d_f)

print theta

theta_2 = np.zeros_like(theta)
theta_2[0] = 1.0 / theta[0]
theta_2[1] = - theta[1] / theta[0]

print theta_2

pwms = np.array([50, 140])
calc_steering = theta[0] * pwms + theta[1]

# enc plot
fig = plt.figure("Steering Angle")
ax_df = fig.add_subplot(1,1,1, ylabel="ax")
ax_df.plot(pwm_entries, d_f, "*", label="d_f")
ax_df.plot(pwms, calc_steering, "-", label="calc_df")
ax_df.legend()
ax_df.grid()

plt.show()

v_dot = 0.
V = 0.
pwm_entries = 0.
C1_pos = 0.
C1_neg = 0.
C2_pos = 0.
C2_neg = 0.
CM = 0.
X = 0.

first = True
positive = True

for file in acc_dir:
	print file
	pathSave = acc_str + file + "/estimator_imu.npz"
	npz_imu = np.load(pathSave)
	psiDot_his    	= npz_imu["psiDot_his"]
	roll_his      	= npz_imu["roll_his"]
	pitch_his     	= npz_imu["pitch_his"]
	yaw_his      	= npz_imu["yaw_his"]
	ax_his      	= npz_imu["ax_his"]
	ay_his      	= npz_imu["ay_his"]
	imu_time  		= npz_imu["imu_time"]

	pathSave = acc_str + file + "/estimator_enc.npz"
	npz_enc = np.load(pathSave)
	v_fl_his 	= npz_enc["v_fl_his"]
	v_fr_his 	= npz_enc["v_fr_his"]
	v_rl_his 	= npz_enc["v_rl_his"]
	v_rr_his 	= npz_enc["v_rr_his"]
	v_meas_his 	= npz_enc["v_meas_his"]
	enc_time  	= npz_enc["enc_time"]

	acc_pwm = int(file.split("_")[1])
	breaking_pwm = int(file.split("_")[2])

	for i in range(len(enc_time) - 1)[::-1]:
		if enc_time[i - 1] == enc_time[i]:
			enc_time = np.delete(enc_time, i)
			v_rr_his = np.delete(v_rr_his, i)

	acc_start = np.argmax(v_rr_his > 0.5)
	acc_max = np.argmax(v_rr_his)
	acc_end = len(v_rr_his) - np.argmax(v_rr_his[:: -1] > 0.6)

	dummy_vrr = copy.copy(v_rr_his[acc_max : acc_end + 1])
	enc_1 = enc_time[acc_max : acc_end + 1]
	dummy_vrr2 = copy.copy(v_rr_his[acc_start : acc_max + 1])
	enc_2 = enc_time[acc_start : acc_max + 1]

	constant_array = []

	"""
	for i in range(len(dummy_vrr2) - 1)[::-1]:
		if dummy_vrr2[i - 1] == dummy_vrr2[i]:
			constant_array.append(len(dummy_vrr2) - i)
			# dummy_vrr2 = np.delete(dummy_vrr2, i)
	"""

	for i in range(len(dummy_vrr2) - 1):
		if dummy_vrr2[i] >= dummy_vrr2[i + 1]:
			constant_array.append(i + 1)
			# dummy_vrr2 = np.delete(dummy_vrr2, i)

	print constant_array

	# group the indeces
	constant_groups = []
	j = 0

	for i in range(len(constant_array)):
		if i == 1:
			constant_groups.append([constant_array[i]])
		elif constant_array[i] - 1 == constant_array[i - 1]:
			constant_groups[j].append(constant_array[i])
		else:
			constant_groups.append([constant_array[i]])
			j += 1

	print constant_groups

	for i in range(len(constant_groups)):
		group = constant_groups[i]
		first_index = group[0] - 1 
		last_index = group[- 1] + 1
		init_value = dummy_vrr2[first_index]
		final_value = dummy_vrr2[last_index]
		init_t = enc_2[first_index]
		final_t = enc_2[last_index]

		delta_value = final_value - init_value
		delta_t = final_t - init_t

		num_constants = len(group)
		for j in range(num_constants):
			delta_t_current = enc_2[group[j]] -init_t
			dummy_vrr2[group[j]] = dummy_vrr2[first_index] + delta_value / delta_t * delta_t_current

	time_series = np.arange(enc_2[0], enc_2[- 1], 0.1)
	vrr2 = np.interp(time_series, enc_2, dummy_vrr2)

	"""
	print time_series
	print vrr2

	pdb.set_trace()

	print enc_time
	print np.diff(enc_time[acc_start : acc_max])
	print np.diff(dummy_vrr2)
	"""

	acc_pos = np.diff(vrr2) / 0.1
	pwm_entries_ = acc_pwm * np.ones_like(acc_pos)
	c1_pos = np.ones_like(acc_pos)

	X_ = np.transpose(np.vstack((pwm_entries_, c1_pos)))

	print X_.shape

	if positive:
		if first:
			v_dot = acc_pos
			X = X_
			pwm_entries = pwm_entries_
			first = False
		else:
			print X_.shape
			print X.shape
			X = np.vstack((X, X_))
			v_dot = np.hstack((v_dot, acc_pos))
			pwm_entries = np.hstack((pwm_entries, pwm_entries_))

	constant_array = []

	for i in range(len(dummy_vrr) - 1):
		if dummy_vrr2[i] <= dummy_vrr[i + 1]:
			constant_array.append(i + 1)
			# dummy_vrr2 = np.delete(dummy_vrr2, i)

	print constant_array

	# group the indeces
	constant_groups = []
	j = 0

	for i in range(len(constant_array)):
		if i == 1:
			constant_groups.append([constant_array[i]])
		elif constant_array[i] - 1 == constant_array[i - 1]:
			constant_groups[j].append(constant_array[i])
		else:
			constant_groups.append([constant_array[i]])
			j += 1

	print constant_groups

	for i in range(len(constant_groups)):
		group = constant_groups[i]
		first_index = group[0] - 1 
		last_index = group[- 1] + 1
		init_value = dummy_vrr[first_index]
		final_value = dummy_vrr[last_index]
		init_t = enc_1[first_index]
		final_t = enc_1[last_index]

		delta_value = final_value - init_value
		delta_t = final_t - init_t

		num_constants = len(group)
		for j in range(num_constants):
			delta_t_current = enc_1[group[j]] - init_t
			dummy_vrr[group[j]] = dummy_vrr[first_index] + delta_value / delta_t * delta_t_current

	time_series_1 = np.arange(enc_1[0], enc_1[- 1], 0.1)
	vrr = np.interp(time_series, enc_1, dummy_vrr)

	acc_neg = np.diff(vrr) / 0.1

	c1_neg = np.ones_like(acc_neg)
	pwm_entries_ = breaking_pwm * np.ones_like(acc_neg)
	X_ = np.transpose(np.vstack((pwm_entries_, c1_neg)))	

	if not positive:
		if first:
			v_dot = acc_neg
			X = X_
			pwm_entries = pwm_entries_
			first = False
		else:
			print X_.shape
			print X.shape
			X = np.vstack((X, X_))
			v_dot = np.hstack((v_dot, acc_neg))
			pwm_entries = np.hstack((pwm_entries, pwm_entries_))

	PLOT = True
	if PLOT:
		"""
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
		print np.mean(ax_his), np.mean(ay_his)
		ax2.legend()
		ax2.grid()
		ax3 = fig.add_subplot(num_plot,1,3,ylabel="pitch & roll angle")
		ax3.plot(imu_time,roll_his,label="roll angle")
		ax3.plot(imu_time,pitch_his,label="pitch angle")
		ax3.legend()
		ax3.grid()
		"""

		# enc plot
		fig = plt.figure("encoder")
		ax4 = fig.add_subplot(1,1,1,ylabel="ax")
		# ax4.plot(enc_time, v_fl_his, "--",	label="fl")
		# ax4.plot(enc_time, v_fr_his, "--",	label="fr")
		# ax4.plot(enc_time, v_rl_his, "-",	label="rl")
		ax4.plot(enc_time, v_rr_his, "b-",	label="rr")
		# ax4.plot(enc_time, savgol_filter(v_rr_his, 19, 2), "r-")
		# ax4.plot(enc_2, dummy_vrr2, "r-")
		ax4.plot(time_series, vrr2, "ko")
		ax4.plot(time_series, vrr2, "k-")
		ax4.plot(enc_time[acc_start], v_rr_his[acc_start], "ro")
		ax4.plot(enc_time[acc_max], v_rr_his[acc_max], "ko")
		ax4.plot(enc_time[acc_end], v_rr_his[acc_end], "mo")

		# ax4.plot(enc_time, v_meas_his, "-",	label="meas")
		ax4.legend()
		ax4.grid()

		plt.show()

theta = np.matmul(np.matmul(np.linalg.inv(np.matmul(np.transpose(X), X)), np.transpose(X)), v_dot)

print theta

pwms = np.array([40, 120])
# Assuming velocity of 1.0
calc_acc = theta[0] * pwms + theta[1] * 1

# enc plot
fig = plt.figure("Acceleration")
ax_df = fig.add_subplot(1,1,1, ylabel="ax")
ax_df.plot(pwm_entries, v_dot, "*", label="d_f")
ax_df.plot(pwms, calc_acc, "-", label="calc_df")
ax_df.legend()
ax_df.grid()

plt.show()