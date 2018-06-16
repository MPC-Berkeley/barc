import numpy as np
import os
import sys
import pdb
import matplotlib.pyplot as plt
homedir = os.path.expanduser("~")
import sys
sys.path.append(sys.path[0]+'/../Utilities')
from trackInitialization import Map
import matplotlib.patches as patches


def main():

    map = Map("oval")
    # FIGURE 1 plotting of estimator output data
    homedir = os.path.expanduser("~")
    pathSave = os.path.join(homedir,"barc_data/estimator_output.npz")
    npz_output = np.load(pathSave)
    x_est_his     		= npz_output["x_est_his"]
    y_est_his     		= npz_output["y_est_his"]
    vx_est_his          = npz_output["vx_est_his"] 
    vy_est_his          = npz_output["vy_est_his"] 
    ax_est_his          = npz_output["ax_est_his"] 
    ay_est_his          = npz_output["ay_est_his"] 
    psiDot_est_his     	= npz_output["psiDot_est_his"]  
    yaw_est_his     	= npz_output["yaw_est_his"]  
    gps_time_his 		= npz_output["gps_time"]
    imu_time_his        = npz_output["imu_time"]
    enc_time_his        = npz_output["enc_time"]
    inp_x_his           = npz_output["inp_x_his"]
    inp_y_his           = npz_output["inp_y_his"]
    inp_v_meas_his      = npz_output["inp_v_meas_his"]
    inp_ax_his          = npz_output["inp_ax_his"]
    inp_ay_his          = npz_output["inp_ay_his"]
    inp_psiDot_his      = npz_output["inp_psiDot_his"]
    inp_a_his           = npz_output["inp_a_his"]
    inp_df_his          = npz_output["inp_df_his"]

    pathSave = os.path.join(homedir,"barc_data/estimator_imu.npz")
    npz_imu = np.load(pathSave)
    psiDot_his    	= npz_imu["psiDot_his"]
    roll_his      	= npz_imu["roll_his"]
    pitch_his     	= npz_imu["pitch_his"]
    yaw_his      	= npz_imu["yaw_his"]
    ax_his      	= npz_imu["ax_his"]
    ay_his      	= npz_imu["ay_his"]
    imu_time  		= npz_imu["imu_time"]

    pathSave = os.path.join(homedir,"barc_data/estimator_gps.npz")
    npz_gps = np.load(pathSave)
    x_his 		= npz_gps["x_his"]
    y_his 		= npz_gps["y_his"]
    gps_time  	= npz_gps["gps_time"]

    pathSave = os.path.join(homedir,"barc_data/estimator_enc.npz")
    npz_enc = np.load(pathSave)
    v_fl_his 	= npz_enc["v_fl_his"]
    v_fr_his 	= npz_enc["v_fr_his"]
    v_rl_his 	= npz_enc["v_rl_his"]
    v_rr_his 	= npz_enc["v_rr_his"]
    enc_time  	= npz_enc["enc_time"]

    pathSave = os.path.join(homedir,"barc_data/estimator_ecu.npz")
    npz_ecu = np.load(pathSave)
    a_his 		= npz_ecu["a_his"]
    df_his 		= npz_ecu["df_his"]
    ecu_time  	= npz_ecu["ecu_time"]

    # Plot Trajectory
    fig = plotTrack(map)

    plt.axis("equal")
    plt.plot(x_est_his,y_est_his,color="green")
    plt.plot(x_his,y_his,color="blue")
    plt.legend()

    plt.show()
    # Acc 
    plt.subplot(311)
    plt.plot(range(0,len(inp_ax_his)), inp_ax_his, '-or')
    plt.plot(range(0,len(ax_est_his)), ax_est_his, '-sb')

    plt.subplot(312)
    plt.plot(range(0,len(inp_ay_his)), inp_ay_his, '-or')
    plt.plot(range(0,len(ay_est_his)), ay_est_his, '-sb')

    plt.subplot(313)
    plt.plot(range(0,len(inp_psiDot_his)), inp_psiDot_his, '-or')
    plt.plot(range(0,len(psiDot_est_his)), psiDot_est_his, '-sb')



    # Check Redundancy
    plt.figure(10)
    plt.plot(range(0,len(gps_time_his)-1), gps_time_his[1:]-gps_time_his[:-1], '--sk')
    plt.ylabel('delta gps time')

    plt.figure(11)
    plt.plot(range(0,len(imu_time_his)-1), imu_time_his[1:]-imu_time_his[:-1], '--sk')
    plt.ylabel('delta imu time')

    plt.figure(12)
    plt.plot(range(0,len(enc_time_his)-1), enc_time_his[1:]-enc_time_his[:-1], '--sk')
    plt.ylabel('delta enc time')

    plt.show()

    # Animation
    fig, axtr, line_est, line_gps, rec = _initializeFigure_xy(map)

    ClosedLoopTraj_gps_x = [] 
    ClosedLoopTraj_gps_y = []
    ClosedLoopTraj_est_x = [] 
    ClosedLoopTraj_est_y = []

    for i in range(len(x_est_his)-1):
        ClosedLoopTraj_gps_x.append(inp_x_his[i])
        ClosedLoopTraj_gps_y.append(inp_y_his[i])
        line_gps.set_data(ClosedLoopTraj_gps_x, ClosedLoopTraj_gps_y)

        ClosedLoopTraj_est_x.append(x_est_his[i+1]) 
        ClosedLoopTraj_est_y.append(y_est_his[i+1])
        line_est.set_data(ClosedLoopTraj_est_x, ClosedLoopTraj_est_y)

        psi = yaw_est_his[i+1]
        x = x_est_his[i+1]
        y = y_est_his[i+1]
        l = 0.4; w = 0.2
        car_x, car_y = getCar_x_y(x, y, psi, l, w)

        rec.set_xy(np.array([car_x, car_y]).T)

        fig.canvas.draw()

def plotTrack(map):
    fig = plt.figure("track x-y plot")
    Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
    Points1 = np.zeros((Points, 2))
    Points2 = np.zeros((Points, 2))
    Points0 = np.zeros((Points, 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')

    return fig


def getCar_x_y(x, y, psi, l, w):
    car_x = [ x + l * np.cos(psi) - w * np.sin(psi), x + l*np.cos(psi) + w * np.sin(psi),
              x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
    car_y = [ y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
              y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]

    return car_x, car_y


def _initializeFigure_xy(map):
    xdata = []; ydata = []
    fig = plt.figure(figsize=(12,8))
    plt.ion()
    axtr = plt.axes()

    Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
    Points1 = np.zeros((Points, 2))
    Points2 = np.zeros((Points, 2))
    Points0 = np.zeros((Points, 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    
    line_est, = axtr.plot(xdata, ydata, '-b')
    line_gps, = axtr.plot(xdata, ydata, '-og')
    
    v = np.array([[ 1.,  1.],
                  [ 1., -1.],
                  [-1., -1.],
                  [-1.,  1.]])

    rec = patches.Polygon(v, alpha=0.5,closed=True, fc='r', ec='k',zorder=10)
    axtr.add_patch(rec)

    plt.show()

    return fig, axtr, line_est, line_gps, rec


if __name__ == '__main__':
    main()