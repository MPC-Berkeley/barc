import numpy as np
import os
import matplotlib.pyplot as plt
import pdb

PLOT = False

l_f = 0.125
l_r = 0.125

homedir = os.path.expanduser("~")

steering_str = homedir + "/barc_debugging/steering/"
acc_str = homedir + "/barc_debugging/acceleration/"
steering_dir = os.listdir(steering_str)
acc_dir = os.listdir(acc_str)

X = 0.
d_f = 0.
pwm_entries = 0.

first = True

for file in steering_dir:
    print file
    pathSave = steering_str + file + "/estimator_imu.npz"
    npz_imu = np.load(pathSave)
    psiDot_his        = npz_imu["psiDot_his"]
    roll_his          = npz_imu["roll_his"]
    pitch_his         = npz_imu["pitch_his"]
    yaw_his          = npz_imu["yaw_his"]
    ax_his          = npz_imu["ax_his"]
    ay_his          = npz_imu["ay_his"]
    imu_time          = npz_imu["imu_time"]

    pathSave = steering_str + file + "/estimator_enc.npz"
    npz_enc = np.load(pathSave)
    v_fl_his     = npz_enc["v_fl_his"]
    v_fr_his     = npz_enc["v_fr_his"]
    v_rl_his     = npz_enc["v_rl_his"]
    v_rr_his     = npz_enc["v_rr_his"]
    v_meas_his     = npz_enc["v_meas_his"]
    enc_time      = npz_enc["enc_time"]

    v_his = 0.5 * (v_rl_his + v_rr_his)
    # v_his = v_rr_his

    """
    pathSave = os.path.join(homedir,"barc_debugging/estimator_ecu.npz")
    npz_ecu = np.load(pathSave)
    a_his         = npz_ecu["a_his"]
    df_his         = npz_ecu["df_his"]
    ecu_time      = npz_ecu["ecu_time"]
    """

    # Calculation of steering angle
    steering_pwm = float(file.split("_")[0])

    mean_vel = np.mean(v_his)
    first_index = np.argmax(v_his > mean_vel)
    enc_time = enc_time[first_index:-25]
    imu_time = imu_time[first_index:-25]
    psi_dot = psiDot_his[first_index:-25]
    v_x = v_his[first_index:-25]

    v_x = np.interp(imu_time, enc_time, v_x)

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
        fig = plt.figure("Imu")
        ax2 = fig.add_subplot(1,1,1,ylabel="IMU acc & psidot")
        ax2.plot(imu_time, psi_dot, label="psiDot")
        ax2.legend()
        ax2.grid()
        
        # enc plot
        fig = plt.figure("encoder")
        ax4 = fig.add_subplot(1,1,1, ylabel="ax")
        # ax4.plot(enc_time, v_fl_his, "--",    label="fl")
        # ax4.plot(enc_time, v_fr_his, "--",    label="fr")
        # ax4.plot(enc_time, v_rl_his, "-",    label="rl")
        ax4.plot(enc_time, v_x, "-", label="rr")
        # ax4.plot(enc_time, v_meas_his, "-",    label="meas")
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
    psiDot_his        = npz_imu["psiDot_his"]
    roll_his          = npz_imu["roll_his"]
    pitch_his         = npz_imu["pitch_his"]
    yaw_his          = npz_imu["yaw_his"]
    ax_his          = npz_imu["ax_his"]
    ay_his          = npz_imu["ay_his"]
    imu_time          = npz_imu["imu_time"]

    pathSave = acc_str + file + "/estimator_enc.npz"
    npz_enc = np.load(pathSave)
    v_fl_his     = npz_enc["v_fl_his"]
    v_fr_his     = npz_enc["v_fr_his"]
    v_rl_his     = npz_enc["v_rl_his"]
    v_rr_his     = npz_enc["v_rr_his"]
    v_meas_his     = npz_enc["v_meas_his"]
    enc_time      = npz_enc["enc_time"]

    acc_pwm = float(file.split("_")[1])
    breaking_pwm = float(file.split("_")[2])

    acc_start = np.argmax(v_rr_his > 0.5)
    acc_max = np.argmax(v_rr_his)
    acc_end = len(v_rr_his) - np.argmax(v_rr_his[:: -1] > 0.6)

    dummy_vrr = v_rr_his[acc_max : acc_end]
    dummy_vrr2 = v_rr_his[acc_start : acc_max]

    for i in range(len(dummy_vrr2 - 1))[::-1]:
        if dummy_vrr2[i - 1] == dummy_vrr2[i]:
            dummy_vrr2 = np.delete(dummy_vrr2, i)

    for i in range(len(dummy_vrr - 1))[::-1]:
        if dummy_vrr[i - 1] == dummy_vrr[i]:
            dummy_vrr = np.delete(dummy_vrr, i)

    acc_pos = np.diff(dummy_vrr2)
    acc_neg = np.diff(dummy_vrr)

    v_pos = dummy_vrr2[:-1]
    v_neg = dummy_vrr[:-1]

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

        # enc plot
        fig = plt.figure("encoder")
        ax4 = fig.add_subplot(1,1,1,ylabel="ax")
        # ax4.plot(enc_time, v_fl_his, "--",    label="fl")
        # ax4.plot(enc_time, v_fr_his, "--",    label="fr")
        # ax4.plot(enc_time, v_rl_his, "-",    label="rl")
        ax4.plot(enc_time, v_rr_his, "-",    label="rr")
        ax4.plot(enc_time[acc_start], v_rr_his[acc_start], "ro")
        ax4.plot(enc_time[acc_max], v_rr_his[acc_max], "ko")
        ax4.plot(enc_time[acc_end], v_rr_his[acc_end], "mo")

        # ax4.plot(enc_time, v_meas_his, "-",    label="meas")
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