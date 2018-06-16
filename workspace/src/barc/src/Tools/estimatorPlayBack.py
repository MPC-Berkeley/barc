import os
import sys
# homedir = os.path.expanduser("~")
# sys.path.append(os.path.join(homedir,"barc/workspace/src/barc/src/Library"))
# from Localization_helpers import Track
import sys
sys.path.append(sys.path[0]+'/../Utilities')
from trackInitialization import Map

from barc.msg import ECU, pos_info, Vel_est
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_imu_fusion
from std_msgs.msg import Header
from numpy import eye, zeros, diag, tan, cos, sin, vstack, linalg, pi
from numpy import ones, polyval, size, dot, add
from scipy.linalg import inv, cholesky
from tf import transformations
import math
import numpy as np
import matplotlib.pyplot as plt    
import pdb
import matplotlib.patches as patches
from trackInitialization import Map



def main():
    # node initialization
    a_delay     = 0.0
    df_delay    = 0.0
    loop_rate   = 50.0
   
    # Red
    Q_noVy = eye(8)
    Q_noVy[0,0] = 0.01 # x
    Q_noVy[1,1] = 0.01 # y
    Q_noVy[2,2] = 0.01 # vx
    Q_noVy[3,3] = 0.01 # vy
    Q_noVy[4,4] = 1.0 # ax
    Q_noVy[5,5] = 1.0 # ay
    Q_noVy[6,6] = 10.0 # psi
    Q_noVy[7,7] = 10.0 # psidot
    # Q[8,8] = 0.0 # psiDot in the model
    R_noVy = eye(6)
    R_noVy[0,0] = 20.0   # x
    R_noVy[1,1] = 20.0   # y
    R_noVy[2,2] = 0.1   # vx
    R_noVy[3,3] = 10.0   # ax
    R_noVy[4,4] = 30.0   # ay
    R_noVy[5,5] = 0.1    # psiDot
    thReset_noVy = 0.8

    # Blue
    Q = eye(8)
    Q[0,0] = 0.01     # x
    Q[1,1] = 0.01     # y
    Q[2,2] = 0.5      # vx
    Q[3,3] = 0.5      # vy
    Q[4,4] = 1.0      # ax
    Q[5,5] = 1.0      # ay
    Q[6,6] = 0.01     # psi
    Q[7,7] = 10.0     # psiDot
    R = eye(7)
    R[0,0] = 0.0005   # x
    R[1,1] = 0.0005   # y
    R[2,2] = 0.1      # vx
    R[3,3] = 0.01     # ax
    R[4,4] = 10.0     # ay
    R[5,5] = 20.0     # psiDot
    R[6,6] = 0.001    # vy
    thReset = 1.4

    # Green
    Q_1 = eye(8)
    Q_1[0,0] = 0.01     # x
    Q_1[1,1] = 0.01     # y
    Q_1[2,2] = 0.5      # vx
    Q_1[3,3] = 0.5      # vy
    Q_1[4,4] = 1.0      # ax
    Q_1[5,5] = 1.0      # ay
    Q_1[6,6] = 0.01     # psi
    Q_1[7,7] = 10.0     # psiDot
    R_1 = eye(7)
    R_1[0,0] = 5.0      # x
    R_1[1,1] = 5.0      # y
    R_1[2,2] = 1.0      # vx
    R_1[3,3] = 0.0001   # ax
    R_1[4,4] = 10.0     # ay
    R_1[5,5] = 20.0     # psiDot
    R_1[6,6] = 0.001    # vy
    thReset_1 = 0.4

    imu = ImuClass(0.0)
    gps = GpsClass(0.0)
    enc = EncClass(0.0)
    ecu = EcuClass(0.0)

    est = EstimatorNoVy(0.0,loop_rate,a_delay,df_delay,Q_noVy,R_noVy, thReset_noVy)
    # est_new = Estimator(0.0,loop_rate,a_delay,df_delay,Q,R, thReset)
    est_new = EstimatorNoVy(0.0,loop_rate,a_delay,df_delay,Q_noVy,R_noVy, 0.5)
    est_new1 = Estimator(0.0,loop_rate,a_delay,df_delay, Q, R, thReset)
    
    onVec = [1, 1, 1]

    map = Map("oval")

    estMsg = pos_info()
    
    # LOAD EXPERIMENT DATA
    homedir = os.path.expanduser("~")
    pathSave = os.path.join(homedir,"barc_data/estimator_output.npz")
    npz_output = np.load(pathSave)
    x_est_his           = npz_output["x_est_his"]
    y_est_his           = npz_output["y_est_his"]
    vx_est_his          = npz_output["vx_est_his"] 
    vy_est_his          = npz_output["vy_est_his"] 
    ax_est_his          = npz_output["ax_est_his"] 
    ay_est_his          = npz_output["ay_est_his"] 
    psiDot_est_his      = npz_output["psiDot_est_his"]  
    yaw_est_his         = npz_output["yaw_est_his"]  
    gps_time            = npz_output["gps_time"]
    imu_time            = npz_output["imu_time"]
    enc_time            = npz_output["enc_time"]
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
    psiDot_his        = npz_imu["psiDot_his"]
    roll_his          = npz_imu["roll_his"]
    pitch_his         = npz_imu["pitch_his"]
    yaw_his          = npz_imu["yaw_his"]
    ax_his          = npz_imu["ax_his"]
    ay_his          = npz_imu["ay_his"]

    pathSave = os.path.join(homedir,"barc_data/estimator_gps.npz")
    npz_gps = np.load(pathSave)
    x_his         = npz_gps["x_his"]
    y_his         = npz_gps["y_his"]

    pathSave = os.path.join(homedir,"barc_data/estimator_enc.npz")
    npz_enc = np.load(pathSave)
    v_fl_his     = npz_enc["v_fl_his"]
    v_fr_his     = npz_enc["v_fr_his"]
    v_rl_his     = npz_enc["v_rl_his"]
    v_rr_his     = npz_enc["v_rr_his"]

    pathSave = os.path.join(homedir,"barc_data/estimator_ecu.npz")
    npz_ecu = np.load(pathSave)
    a_his         = npz_ecu["a_his"]
    df_his        = npz_ecu["df_his"]

    fig = plotTrack(map)
    # ax1.plot(l.nodes[0],l.nodes[1],color="grey",linestyle="--", alpha=0.3)
    # ax1.plot(l.nodes_bound1[0],l.nodes_bound1[1],color="red",alpha=0.3)
    # ax1.plot(l.nodes_bound2[0],l.nodes_bound2[1],color="red",alpha=0.3)
    plt.axis("equal")
    plt.plot(x_his,y_his,color="blue")

    plt.show()
    flagHalfLap = False
    plotDebug = False


    if plotDebug == True:
        fig, axtr, line_tr, line_tr_new, line_tr_new1, line_pred, line_SS, line_cl, line_cl_new, line_cl_new1, line_gps_cl, rec, rec_new, rec_new1, recEst = _initializeFigure_xy(map, onVec)

    ClosedLoopTraj_gps_x = [] 
    ClosedLoopTraj_gps_y = []

    ClosedLoopTraj_x = []
    ClosedLoopTraj_y = []
    ClosedLoopTraj_x_new = []
    ClosedLoopTraj_y_new = []
    ClosedLoopTraj_x_new1 = []
    ClosedLoopTraj_y_new1 = []

    maxVx = 0

    for i in range(len(inp_ax_his)-1):
        # READ DATA INPUT TO ESTIMATOR
        gps.x       = inp_x_his[i]
        gps.y       = inp_y_his[i]
        imu.ax      = inp_ax_his[i]
        imu.ay      = inp_ay_his[i]
        imu.psiDot  = inp_psiDot_his[i]
        enc.v_meas  = inp_v_meas_his[i]
        ecu.a       = inp_a_his[i]
        ecu.df      = inp_df_his[i]

        est.estimateState(imu,gps,enc,ecu,est.ekf)
        est_new.estimateState(imu,gps,enc,ecu,est_new.ekf)
        est_new1.estimateState(imu,gps,enc,ecu,est_new1.ekf)

        if plotDebug == True:
            estimatedStates = np.array([est.vx_est, est.vy_est, est.psiDot_est, est.yaw_est, est.x_est, est.y_est])
            estimatedStates_new = np.array([est_new.vx_est, est_new.vy_est, est_new.psiDot_est, est_new.yaw_est, est_new.x_est, est_new.y_est])
            estimatedStates_new1 = np.array([est_new1.vx_est, est_new1.vy_est, est_new1.psiDot_est, est_new1.yaw_est, est_new1.x_est, est_new1.y_est])

            s, ey, epsi, insideMap = map.getLocalPosition(estimatedStates[4], estimatedStates[5], estimatedStates[3])

            if s > map.TrackLength / 2:
                flagHalfLap = True

            if (s < map.TrackLength / 4) and (flagHalfLap == True): # New lap
                ClosedLoopTraj_gps_x = []
                ClosedLoopTraj_gps_y = []
                ClosedLoopTraj_x = []
                ClosedLoopTraj_y = []
                flagHalfLap = False

            x = estimatedStates[4]
            y = estimatedStates[5]

            x_new = estimatedStates_new[4]
            y_new = estimatedStates_new[5]

            x_new1 = estimatedStates_new1[4]
            y_new1 = estimatedStates_new1[5]

            ClosedLoopTraj_gps_x.append(gps.x) 
            ClosedLoopTraj_gps_y.append(gps.y)

            if onVec[0] == 1:
                ClosedLoopTraj_x.append(est.x_est) 
                ClosedLoopTraj_y.append(est.y_est)
                psi = estimatedStates[3]
                l = 0.4; w = 0.2
                car_x, car_y = getCar_x_y(x, y, psi, l, w)

                line_cl.set_data(ClosedLoopTraj_x, ClosedLoopTraj_y)
                line_tr.set_data(estimatedStates[4], estimatedStates[5])
                rec.set_xy(np.array([car_x, car_y]).T)

            if onVec[1] == 1:
                ClosedLoopTraj_x_new.append(est_new.x_est) 
                ClosedLoopTraj_y_new.append(est_new.y_est)
                psi_new = estimatedStates_new[3]
                l = 0.4; w = 0.2
                car_x_new, car_y_new = getCar_x_y(x_new, y_new, psi_new, l, w)

                line_cl_new.set_data(ClosedLoopTraj_x_new, ClosedLoopTraj_y_new)
                line_tr_new.set_data(estimatedStates_new[4], estimatedStates_new[5])
                rec_new.set_xy(np.array([car_x_new, car_y_new]).T)


            if onVec[2] == 1:
                ClosedLoopTraj_x_new1.append(est_new1.x_est) 
                ClosedLoopTraj_y_new1.append(est_new1.y_est)
                psi_new1 = estimatedStates_new1[3]
                l = 0.4; w = 0.2
                car_x_new1, car_y_new1 = getCar_x_y(x_new1, y_new1, psi_new1, l, w)

                line_cl_new1.set_data(ClosedLoopTraj_x_new1, ClosedLoopTraj_y_new1)
                line_tr_new1.set_data(estimatedStates_new1[4], estimatedStates_new1[5])
                rec_new1.set_xy(np.array([car_x_new1, car_y_new1]).T)
            
            car_xEst, car_yEst = getCar_x_y(x_est_his[i], y_est_his[i], yaw_est_his[i], l, w)
            recEst.set_xy(np.array([car_xEst, car_yEst]).T)

            line_gps_cl.set_data(ClosedLoopTraj_gps_x, ClosedLoopTraj_gps_y)
            maxVx = np.maximum(maxVx, estimatedStates[0])

            StringValue = "vx: "+str(estimatedStates[0])+" max vx: "+str(maxVx)+"psiDot: "+str(imu.psiDot)
            axtr.set_title(StringValue)
            
            if insideMap == 1:
                fig.canvas.draw()


    homedir = os.path.expanduser("~")
    pathSave = os.path.join(homedir,"barc_estimator_play_back/estimator_output.npz")
    np.savez(pathSave,yaw_est_his       = est.yaw_est_his,
                      psiDot_est_his    = est.psiDot_est_his,
                      x_est_his         = est.x_est_his,
                      y_est_his         = est.y_est_his,
                      vx_est_his        = est.vx_est_his,
                      vy_est_his        = est.vy_est_his,
                      ax_est_his        = est.ax_est_his,
                      ay_est_his        = est.ay_est_his,
                      estimator_time    = est.time_his,
                      inp_x_his         = est.x_his,
                      inp_y_his         = est.y_his,
                      inp_v_meas_his    = est.v_meas_his,
                      inp_ax_his        = est.ax_his,
                      inp_ay_his        = est.ay_his,
                      inp_psiDot_his    = est.psiDot_his,
                      inp_a_his         = est.a_his,
                      inp_df_his        = est.df_his)

    pathSave = os.path.join(homedir,"barc_estimator_play_back/estimator_imu.npz")
    np.savez(pathSave,psiDot_his    = imu.psiDot_his,
                      roll_his      = imu.roll_his,
                      pitch_his     = imu.pitch_his,
                      yaw_his       = imu.yaw_his,
                      ax_his        = imu.ax_his,
                      ay_his        = imu.ay_his,
                      imu_time      = imu.time_his)

    pathSave = os.path.join(homedir,"barc_estimator_play_back/estimator_gps.npz")
    np.savez(pathSave,x_his         = gps.x_his,
                      y_his         = gps.y_his,
                      gps_time      = gps.time_his)

    pathSave = os.path.join(homedir,"barc_estimator_play_back/estimator_enc.npz")
    np.savez(pathSave,v_fl_his          = enc.v_fl_his,
                      v_fr_his          = enc.v_fr_his,
                      v_rl_his          = enc.v_rl_his,
                      v_rr_his          = enc.v_rr_his,
                      enc_time          = enc.time_his)

    pathSave = os.path.join(homedir,"barc_estimator_play_back/estimator_ecu.npz")
    np.savez(pathSave,a_his         = ecu.a_his,
                      df_his        = ecu.df_his,
                      ecu_time      = ecu.time_his)

    print "Finishing saveing state estimation data"

    # Acc 
    plt.subplot(311)
    plt.plot(range(0,len(inp_ax_his)), inp_ax_his, '-o')
    plt.plot(range(0,len(ax_est_his)), ax_est_his, '-sb')
    plt.plot(range(0,len(est.ax_est_his)), est.ax_est_his, '-or')
    plt.plot(range(0,len(est_new.ax_est_his)), est_new.ax_est_his, '-ob')
    plt.plot(range(0,len(est_new1.ax_est_his)), est_new1.ax_est_his, '--og')

    plt.subplot(312)
    plt.plot(range(0,len(inp_ay_his)), inp_ay_his, '-or')
    plt.plot(range(0,len(ay_est_his)), ay_est_his, '-sb')
    plt.plot(range(0,len(est.ay_est_his)), est.ay_est_his, '-or')
    plt.plot(range(0,len(est_new.ay_est_his)), est_new.ay_est_his, '-ob')
    plt.plot(range(0,len(est_new1.ay_est_his)), est_new1.ay_est_his, '--og')

    plt.subplot(313)
    plt.plot(range(0,len(inp_psiDot_his)), inp_psiDot_his, '-or')
    plt.plot(range(0,len(psiDot_est_his)), psiDot_est_his, '-sb')
    plt.plot(range(0,len(est.psiDot_est_his)), est.psiDot_est_his, '-or')
    plt.plot(range(0,len(est_new.psiDot_est_his)), est_new.psiDot_est_his, '-ob')
    plt.plot(range(0,len(est_new1.psiDot_est_his)), est_new1.psiDot_est_his, '--og')

    plt.show()

    xmin = 5000 #2000 #0
    xmax = 5200 #2660 #len(est.vx_est_his)
    # xmin = 0
    # xmax = len(est.vx_est_his)

    fig = plotTrack(map)
    # plt.plot(x_est_his[xmin:xmax],y_est_his[xmin:xmax],"--ob")

    # fig = plotTrack(map)
    axes = plt.gca()
    if onVec[0] == 1:
        plt.plot(est.x_est_his[xmin:xmax], est.y_est_his[xmin:xmax], '-or')
    if onVec[1] == 1:
        plt.plot(est_new.x_est_his[xmin:xmax], est_new.y_est_his[xmin:xmax], '-sb')
    if onVec[2] == 1:
        plt.plot(est_new1.x_est_his[xmin:xmax], est_new1.y_est_his[xmin:xmax], '--og')
    plt.plot(x_est_his[xmin:xmax], y_est_his[xmin:xmax], '--sk')
    # axes.set_xlim([xmin,xmax])
    # plt.ylabel('y')    

    plt.figure(10)
    axes = plt.gca()
    if onVec[0] == 1:
        plt.plot(range(xmin,xmax), est.vx_est_his[xmin:xmax], '-or')
    if onVec[1] == 1:
        plt.plot(range(xmin,xmax), est_new.vx_est_his[xmin:xmax], '-sb')
    if onVec[2] == 1:
        plt.plot(range(xmin,xmax), est_new1.vx_est_his[xmin:xmax], '--og')
    plt.plot(range(xmin,xmax), vx_est_his[xmin:xmax], '--sk')
    axes.set_xlim([xmin,xmax])
    plt.ylabel('vx')

    plt.figure(11)
    axes = plt.gca()
    if onVec[0] == 1:
        plt.plot(range(xmin,xmax), est.vy_est_his[xmin:xmax], '-or')
    if onVec[1] == 1:
        plt.plot(range(xmin,xmax), est_new.vy_est_his[xmin:xmax], '-sb')
    if onVec[2] == 1:
        plt.plot(range(xmin,xmax), est_new1.vy_est_his[xmin:xmax], '--og')
    plt.plot(range(xmin,xmax), vy_est_his[xmin:xmax], '--sk')
    axes.set_xlim([xmin,xmax])
    plt.ylabel('vy')

    plt.figure(12)
    axes = plt.gca()
    if onVec[0] == 1:
        plt.plot(range(xmin,xmax), est.psiDot_est_his[xmin:xmax], '-or')
    if onVec[1] == 1:
        plt.plot(range(xmin,xmax), est_new.psiDot_est_his[xmin:xmax], '-sb')
    if onVec[2] == 1:
        plt.plot(range(xmin,xmax), est_new1.psiDot_est_his[xmin:xmax], '--og')
    plt.plot(range(xmin,xmax), psiDot_est_his[xmin:xmax], '--sk')
    axes.set_xlim([xmin,xmax])
    plt.ylabel('wz')

    plt.figure(13)
    axes = plt.gca()
    if onVec[0] == 1:
        plt.plot(range(xmin,xmax), est.yaw_est_his[xmin:xmax], '-or')
    if onVec[1] == 1:
        plt.plot(range(xmin,xmax), est_new.yaw_est_his[xmin:xmax], '-sb')
    if onVec[2] == 1:
        plt.plot(range(xmin,xmax), est_new1.yaw_est_his[xmin:xmax], '--og')
    plt.plot(range(xmin,xmax), yaw_est_his[xmin:xmax], '--sk')
    axes.set_xlim([xmin,xmax])
    plt.ylabel('psi')

    plt.figure(14)
    axes = plt.gca()
    if onVec[0] == 1:
        plt.plot(range(xmin,xmax), est.x_est_his[xmin:xmax], '-or')
    if onVec[1] == 1:
        plt.plot(range(xmin,xmax), est_new.x_est_his[xmin:xmax], '-sb')
    if onVec[2] == 1:
        plt.plot(range(xmin,xmax), est_new1.x_est_his[xmin:xmax], '--og')
    plt.plot(range(xmin,xmax), x_est_his[xmin:xmax], '--sk')
    axes.set_xlim([xmin,xmax])
    plt.ylabel('x')

    plt.figure(15)
    axes = plt.gca()
    if onVec[0] == 1:
        plt.plot(range(xmin,xmax), est.y_est_his[xmin:xmax], '-or')
    if onVec[1] == 1:
        plt.plot(range(xmin,xmax), est_new.y_est_his[xmin:xmax], '-sb')
    if onVec[2] == 1:
        plt.plot(range(xmin,xmax), est_new1.y_est_his[xmin:xmax], '--og')
    plt.plot(range(xmin,xmax), y_est_his[xmin:xmax], '--sk')
    axes.set_xlim([xmin,xmax])
    plt.ylabel('y')

    pdb.set_trace()

    plt.show()

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

def _initializeFigure_xy(map, onVec):
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
    line_cl, = axtr.plot(xdata, ydata, '-k')
    line_cl_new, = axtr.plot(xdata, ydata, '-k')
    line_cl_new1, = axtr.plot(xdata, ydata, '-k')
    line_gps_cl, = axtr.plot(xdata, ydata, '--og')
    line_tr, = axtr.plot(xdata, ydata, '-or')
    line_tr_new, = axtr.plot(xdata, ydata, '-or')
    line_tr_new1, = axtr.plot(xdata, ydata, '-or')
    line_SS, = axtr.plot(xdata, ydata, 'og')
    line_pred, = axtr.plot(xdata, ydata, '-or')
    
    v = np.array([[ 1.,  1.],
                  [ 1., -1.],
                  [-1., -1.],
                  [-1.,  1.]])

    rec = patches.Polygon(v, alpha=0.5,closed=True, fc='r', ec='k',zorder=10)
    rec_new = patches.Polygon(v, alpha=0.5,closed=True, fc='b', ec='k',zorder=10)
    rec_new1 = patches.Polygon(v, alpha=0.5,closed=True, fc='g', ec='k',zorder=10)
    recEst = patches.Polygon(v, alpha=0.5,closed=True, fc='y', ec='k',zorder=10)
    if onVec[0] == 1:
        axtr.add_patch(rec)
    if onVec[1] == 1:
        axtr.add_patch(rec_new)
    if onVec[2] == 1:
        axtr.add_patch(rec_new1)

    axtr.add_patch(recEst)

    plt.show()

    return fig, axtr, line_tr, line_tr_new, line_tr_new1, line_pred, line_SS, line_cl, line_cl_new, line_cl_new1, line_gps_cl, rec, rec_new, rec_new1, recEst

class EstimatorNoVy(object):
    def __init__(self,t0,loop_rate,a_delay,df_delay,Q,R,thReset):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        self.thReset = thReset
        dt          = 1.0 / loop_rate
        L_f         = 0.125       # distance from CoG to front axel
        L_r         = 0.125       # distance from CoG to rear axel
        self.vhMdl  = (L_f, L_r)
        self.Q      = Q
        self.R      = R
        self.P      = np.eye(np.size(Q,0)) # initializationtial covariance matrix
        self.z      = np.zeros(np.size(Q,0)) # initial state mean
        self.dt     = dt
        self.a_delay        = a_delay
        self.df_delay       = df_delay
        self.a_his          = [0.0]*int(a_delay/dt)
        self.df_his         = [0.0]*int(df_delay/dt)

        self.t0             = t0

        self.x_est          = 0.0
        self.y_est          = 0.0
        self.vx_est         = 0.0
        self.vy_est         = 0.0
        self.v_est          = 0.0
        self.ax_est         = 0.0
        self.ay_est         = 0.0
        self.yaw_est        = 0.0
        self.psiDot_est     = 0.0
        self.psiDot_drift_est = 0.0

        self.x_est_his          = []
        self.y_est_his          = []
        self.vx_est_his         = []
        self.vy_est_his         = []
        self.v_est_his          = []
        self.ax_est_his         = []
        self.ay_est_his         = []
        self.yaw_est_his        = []
        self.psiDot_est_his     = []
        self.time_his           = []

        # SAVE THE measurement/input SEQUENCE USED BY KF
        self.x_his      = []
        self.y_his      = []
        self.v_meas_his = []
        self.ax_his     = []
        self.ay_his     = []
        self.psiDot_his = []
        self.a_his      = []
        self.df_his     = []
    # ecu command update
    def estimateState(self,imu,gps,enc,ecu,KF):
        self.a_his.append(ecu.a)
        self.df_his.append(ecu.df)
        u = [self.a_his.pop(0), self.df_his.pop(0)]
        # u = [ecu.a, self.df_his.pop(0)]
        
        bta = 0.5 * u[1]
        y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, imu.psiDot, sin(bta)*enc.v_meas])
        y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, imu.psiDot])

        if np.abs(imu.psiDot) < self.thReset:
            self.z[3] = 0.0001

        KF(y,u)

        imu.saveHistory()
        gps.saveHistory()
        enc.saveHistory()
        ecu.saveHistory()
        self.saveHistory()
        
    def ekf(self, y, u):
        
        xDim    = self.z.size                           # dimension of the state
        mx_kp1  = self.f(self.z, u)                     # predict next state
        A       = self.numerical_jac(self.f, self.z, u) # linearize process model about current state
        P_kp1   = dot(dot(A,self.P),A.T) + self.Q           # proprogate variance
        my_kp1  = self.h(mx_kp1, u)                              # predict future output
        H       = self.numerical_jac(self.h, mx_kp1, u)     # linearize measurement model about predicted next state
        P12     = dot(P_kp1, H.T)                           # cross covariance
        K       = dot(P12, inv( dot(H,P12) + self.R))       # Kalman filter gain
        

        self.z  = mx_kp1 + dot(K,(y - my_kp1))
        # if np.abs(y[5]) < 0.5:
        #     self.z[3] = 0

        self.P  = dot(dot(K,self.R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )

        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z


    def ukf(self, y, u):
       
        xDim        = self.z.size
        sqrtnP      = cholesky(xDim*self.P)
        sm_km1      = list(add(self.z,sqrtnP))
        sm_km1.extend(list(add(self.z,-sqrtnP)))

        # prior update
        sx_k = [self.f(s, u) for s in sm_km1]
        mx_k = 1.0/len(sx_k)*sum(sx_k)
        P_m  = self.Q + 1.0/len(sx_k)*sum([np.outer((sx-mx_k),(sx-mx_k)) for sx in sx_k])

        # posterior update
        sy_k = [self.h(s, u) for s in sx_k]
        my_k = 1.0/len(sy_k)*sum(sy_k)
        P_zz  = self.R + 1.0/len(sy_k)*sum([np.outer((sy-my_k),(sy-my_k)) for sy in sy_k])

        # cross covariance
        P_xz = 1.0/len(sy_k)*sum([np.outer((sx_k[i]-mx_k),(sy_k[i]-my_k)) for i in range(len(sy_k))])

        # Kalman filter
        K = dot(P_xz,inv(P_zz))
        self.z = mx_k + dot(K, y-my_k)
        self.P = P_m - dot(K, dot(P_zz, K.T))
        
        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z

    def numerical_jac(self,func,x,u):
       
        y = func(x,u)
        
        jac = zeros( (y.size,x.size) )
        eps = 1e-5
        xp = np.copy(x)
        
        for i in range(x.size):
            xp[i] = x[i] + eps/2.0
            yhi = func(xp, u)
            xp[i] = x[i] - eps/2.0
            ylo = func(xp, u)
            xp[i] = x[i]
            jac[:,i] = (yhi - ylo) / eps
        return jac

    def f(self, z, u):
        """ This Sensor model contains a pure Sensor-Model and a Kinematic model. They're independent from each other."""
        dt = self.dt
        zNext = [0]*8
        zNext[0] = z[0] + dt*(cos(z[6])*z[2] - sin(z[6])*z[3])  # x
        zNext[1] = z[1] + dt*(sin(z[6])*z[2] + cos(z[6])*z[3])  # y
        zNext[2] = z[2] + dt*(z[4]+z[7]*z[3])                   # v_x
        zNext[3] = z[3] + dt*(z[5]-z[7]*z[2])                   # v_y
        zNext[4] = z[4]                                         # a_x
        zNext[5] = z[5]                                         # a_y
        zNext[6] = z[6] + dt*(z[7])                             # psi
        zNext[7] = z[7]                                         # psidot
        # zNext[8] = z[8]                                         # psidot_drift
        return np.array(zNext)

    def h(self, x, u):
        """ This is the measurement model to the kinematic<->sensor model above """
        y = [0]*6
        y[0] = x[0]      # x
        y[1] = x[1]      # y
        y[2] = x[2]      # vx
        y[3] = x[4]      # a_x
        y[4] = x[5]      # a_y
        # y[5] = x[7]+x[8] # psiDot
        y[5] = x[7] # psiDot
        # y[6] = x[3]      # vy
        return np.array(y)

    def saveHistory(self):

        self.x_est_his.append(self.x_est)
        self.y_est_his.append(self.y_est)
        self.vx_est_his.append(self.vx_est)
        self.vy_est_his.append(self.vy_est)
        self.v_est_his.append(self.v_est)
        self.ax_est_his.append(self.ax_est)
        self.ay_est_his.append(self.ay_est)
        self.yaw_est_his.append(self.yaw_est)
        self.psiDot_est_his.append(self.psiDot_est)

class Estimator(object):
    """ Object collecting  estimated state data
    Attributes:
        Estimated states:
            1.x_est     2.y_est
            3.vx_est    4.vy_est        5.v_est
            6.ax_est    7.ay_est
            8.yaw_est   9.psiDot_est    10.psiDrift_est
        Estimated states history:
            1.x_est_his     2.y_est_his
            3.vx_est_his    4.vy_est_his        5.v_est_his
            6.ax_est_his    7.ay_est_his
            8.yaw_est_his   9.psiDot_est_his    10.psiDrift_est_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    Methods:
        stateEstimate(imu,gps,enc,ecu):
            Estimate current state from sensor data
        ekf(y,u):
            Extended Kalman filter
        ukf(y,u):
            Unscented Kalman filter
        numerical_jac(func,x,u):
            Calculate jacobian numerically
        f(x,u):
            System prediction model
        h(x,u):
            System measurement model
    """

    def __init__(self,t0,loop_rate,a_delay,df_delay,Q,R,thReset):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        self.thReset = thReset

        dt          = 1.0 / loop_rate
        self.rate   = 50
        L_f         = 0.125       # distance from CoG to front axel
        L_r         = 0.125       # distance from CoG to rear axel
        self.vhMdl  = (L_f, L_r)
        self.Q      = Q
        self.R      = R
        self.P      = np.eye(np.size(Q,0)) # initializationtial covariance matrix
        self.z      = np.zeros(np.size(Q,0)) # initial state mean
        self.dt     = dt
        self.a_delay        = a_delay
        self.df_delay       = df_delay
        self.a_his          = [0.0]*int(a_delay/dt)
        self.df_his         = [0.0]*int(df_delay/dt)

        self.t0             = t0

        self.x_est          = 0.0
        self.y_est          = 0.0
        self.vx_est         = 0.0
        self.vy_est         = 0.0
        self.v_est          = 0.0
        self.ax_est         = 0.0
        self.ay_est         = 0.0
        self.yaw_est        = 0.0
        self.psiDot_est     = 0.0
        self.psiDrift_est   = 0.0
        self.curr_time      = 0.0

        self.x_est_his          = []
        self.y_est_his          = []
        self.vx_est_his         = []
        self.vy_est_his         = []
        self.v_est_his          = []
        self.ax_est_his         = []
        self.ay_est_his         = []
        self.yaw_est_his        = []
        self.psiDot_est_his     = []
        self.time_his           = []

        # SAVE THE measurement/input SEQUENCE USED BY KF
        self.x_his      = []
        self.y_his      = []
        self.v_meas_his = []
        self.ax_his     = []
        self.ay_his     = []
        self.psiDot_his = []
        self.inp_a_his  = []
        self.inp_df_his = []

        self.gps_time = []
        self.enc_time = []
        self.imu_time = []

        self.oldGPS_x = 0.0
        self.oldGPS_y = 0.0

    # ecu command update
    def estimateState(self,imu,gps,enc,ecu,KF):
        """Do extended Kalman filter to estimate states"""
        self.curr_time = 0.0

        self.a_his.append(ecu.a)
        self.df_his.append(ecu.df)
        u = [self.a_his.pop(0), self.df_his.pop(0)]
        # u = [ecu.a, self.df_his.pop(0)]
        
        bta = 0.5 * u[1]
        dist   = np.sqrt(( self.x_est - gps.x )**2 + ( self.y_est - gps.y )**2)

        # if ( dist >= 1 ) or ( (gps.x == self.oldGPS_x) and (gps.x == self.oldGPS_y) ):
        # if ( (gps.x == self.oldGPS_x) and (gps.x == self.oldGPS_y) ):
        if 0 == 1:
            modeGPS = False
            y = np.array([enc.v_meas, imu.ax, imu.ay, imu.psiDot, bta * enc.v_meas])
        else:
            modeGPS = True
            y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, imu.psiDot, bta * enc.v_meas])


        self.oldGPS_x = gps.x
        self.oldGPS_y = gps.y

        if np.abs(imu.psiDot) < self.thReset:
            self.z[3] = 0

        KF(y,u, modeGPS)


        # SAVE THE measurement/input SEQUENCE USED BY KF
        self.x_his.append(gps.x)
        self.y_his.append(gps.y)
        self.v_meas_his.append(enc.v_meas)
        self.ax_his.append(imu.ax)
        self.ay_his.append(imu.ay)
        self.psiDot_his.append(imu.psiDot)
        self.inp_a_his.append(u[0])
        self.inp_df_his.append(u[1])

        # SAVE output KF given the above measurements
        self.saveHistory()

    def ekf(self, y, u, modeGPS):
        """
        EKF   Extended Kalman Filter for nonlinear dynamic systems
        ekf(f,mx,P,h,z,Q,R) returns state estimate, x and state covariance, P 
        for nonlinear dynamic system:
                  x_k+1 = f(x_k) + w_k
                  y_k   = h(x_k) + v_k
        where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
              v ~ N(0,R) meaning v is gaussian noise with covariance R
        Inputs:    f: function handle for f(x)
                   z_EKF: "a priori" state estimate
                   P: "a priori" estimated state covariance
                   h: fanction handle for h(x)
                   y: current measurement
                   Q: process noise covariance 
                   R: measurement noise covariance
                   args: additional arguments to f(x, *args)
        Output:    mx_kp1: "a posteriori" state estimate
                   P_kp1: "a posteriori" state covariance
                   
        Notation: mx_k = E[x_k] and my_k = E[y_k], where m stands for "mean of"
        """
        
        xDim    = self.z.size                           # dimension of the state

        mx_kp1  = self.f(self.z, u)                               # predict next state
        A       = self.numerical_jac(self.f, self.z, u, modeGPS)  # linearize process model about current state

        P_kp1   = dot(dot(A,self.P),A.T) + self.Q                 # proprogate variance

        my_kp1  = self.h(mx_kp1, u, modeGPS)                      # predict future output
        H       = self.numerical_jac(self.h, mx_kp1, u, modeGPS)  # linearize measurement model about predicted next state
        
        P12     = dot(P_kp1, H.T)                                 # cross covariance

        if modeGPS == True:
            K       = dot(P12, inv( dot(H,P12) + self.R))       # Kalman filter gain
        else:
            K       = dot(P12, inv( dot(H,P12) + self.R[2:,2:]))       # Kalman filter gain
            
        self.z  = mx_kp1 + dot(K,(y - my_kp1))

        if modeGPS == True:
            self.P  = dot(dot(K,self.R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )
        else:
            self.P  = dot(dot(K,self.R[2:,2:]),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )

        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z


    def ukf(self, y, u):
        """
        UKF   Unscented Kalman Filter for nonlinear dynamic systems
        ekf(f,mx,P,h,z,Q,R) returns state estimate, x and state covariance, P 
        for nonlinear dynamic system:
                  x[k] = f(x[k-1],u[k-1]) + v[k-1]
                  y[k] = h(x[k]) + w[k]
        where v ~ N(0,Q) meaning v is gaussian noise with covariance Q
              w ~ N(0,R) meaning w is gaussian noise with covariance R
        Inputs:    f: function handle for f(x)
                   h: function handle for h(x)
                   y: current measurement
                   Q: process noise covariance 
                   R: measurement noise covariance
        Output:    mx_k: "a posteriori" state estimate
                   P_k: "a posteriori" state covariance
                   
        Notation: mx_k = E[x_k] and my_k = E[y_k], where m stands for "mean of"
        """

        # sigma-points: generate a list, "sm_km1"
        xDim        = self.z.size
        sqrtnP      = cholesky(xDim*self.P)
        sm_km1      = list(add(self.z,sqrtnP))
        sm_km1.extend(list(add(self.z,-sqrtnP)))

        # prior update
        sx_k = [self.f(s, u) for s in sm_km1]
        mx_k = 1.0/len(sx_k)*sum(sx_k)
        P_m  = self.Q + 1.0/len(sx_k)*sum([np.outer((sx-mx_k),(sx-mx_k)) for sx in sx_k])

        # posterior update
        sy_k = [self.h(s, u) for s in sx_k]
        my_k = 1.0/len(sy_k)*sum(sy_k)
        P_zz  = self.R + 1.0/len(sy_k)*sum([np.outer((sy-my_k),(sy-my_k)) for sy in sy_k])

        # cross covariance
        P_xz = 1.0/len(sy_k)*sum([np.outer((sx_k[i]-mx_k),(sy_k[i]-my_k)) for i in range(len(sy_k))])

        # Kalman filter
        K = dot(P_xz,inv(P_zz))
        self.z = mx_k + dot(K, y-my_k)
        self.P = P_m - dot(K, dot(P_zz, K.T))
        
        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z

    def numerical_jac(self,func,x,u, modeGPS):
        """
        Function to compute the numerical jacobian of a vector valued function 
        using final differences
        """
        # numerical gradient and diagonal hessian
        y = func(x,u, modeGPS)
        
        jac = zeros( (y.size,x.size) )
        eps = 1e-5
        xp = np.copy(x)
        
        for i in range(x.size):
            xp[i] = x[i] + eps/2.0
            yhi = func(xp, u, modeGPS)
            xp[i] = x[i] - eps/2.0
            ylo = func(xp, u, modeGPS)
            xp[i] = x[i]
            jac[:,i] = (yhi - ylo) / eps
        return jac

    def f(self, z, u, modeGPS=True):
        """ This Sensor model contains a pure Sensor-Model and a Kinematic model. They're independent from each other."""
        dt = self.dt
        zNext = [0]*8
        zNext[0] = z[0] + dt*(cos(z[6])*z[2] - sin(z[6])*z[3])  # x
        zNext[1] = z[1] + dt*(sin(z[6])*z[2] + cos(z[6])*z[3])  # y
        zNext[2] = z[2] + dt*(z[4]+z[7]*z[3])                   # v_x
        zNext[3] = z[3] + dt*(z[5]-z[7]*z[2])                   # v_y
        zNext[4] = z[4]                                         # a_x
        zNext[5] = z[5]                                         # a_y
        zNext[6] = z[6] + dt*(z[7])                             # psi
        zNext[7] = z[7]                                         # psidot
        return np.array(zNext)

    def h(self, x, u, modeGPS):
        """ This is the measurement model to the kinematic<->sensor model above """
        if modeGPS:
            y = [0]*7
            y[0] = x[0]   # x
            y[1] = x[1]   # y
            y[2] = x[2]   # vx
            y[3] = x[4]   # a_x
            y[4] = x[5]   # a_y
            y[5] = x[7]   # psiDot
            y[6] = x[3]   # vy
        else:
            y = [0]*5
            y[0] = x[2]   # vx
            y[1] = x[4]   # a_x
            y[2] = x[5]   # a_y
            y[3] = x[7]   # psiDot
            y[4] = x[3]   # vy
        return np.array(y)

    def saveHistory(self):
        self.time_his.append(self.curr_time)

        self.x_est_his.append(self.x_est)
        self.y_est_his.append(self.y_est)
        self.vx_est_his.append(self.vx_est)
        self.vy_est_his.append(self.vy_est)
        self.v_est_his.append(self.v_est)
        self.ax_est_his.append(self.ax_est)
        self.ay_est_his.append(self.ay_est)
        self.yaw_est_his.append(self.yaw_est)
        self.psiDot_est_his.append(self.psiDot_est)


class ImuClass(object):
    
    def __init__(self,t0):
        
        # Imu measurement
        self.yaw     = 0.0
        self.psiDot  = 0.0
        self.ax      = 0.0
        self.ay      = 0.0
        self.roll    = 0.0
        self.pitch   = 0.0
        
        # Imu measurement history
        self.yaw_his     = []
        self.psiDot_his  = []
        self.ax_his      = []
        self.ay_his      = []
        self.roll_his    = []
        self.pitch_his   = []
        
        # time stamp
        self.t0          = t0
        self.time_his    = []


    def imu_callback(self,data):
        """Unpack message from sensor, IMU"""
        
        self.yaw += self.psiDot * 0.02
   
        ori = data.orientation
        quaternion = (ori.x, ori.y, ori.z, ori.w)
        (roll_raw, pitch_raw, dummy) = transformations.euler_from_quaternion(quaternion)
        self.roll   = roll_raw
        self.pitch  = pitch_raw

        w_z = data.angular_velocity.z
        a_x = data.linear_acceleration.x
        a_y = data.linear_acceleration.y
        a_z = data.linear_acceleration.z

        self.psiDot = w_z
        # Transformation from imu frame to vehicle frame (negative roll/pitch and reversed matrix multiplication to go back)
        self.ax = cos(-pitch_raw)*a_x + sin(-pitch_raw)*sin(-roll_raw)*a_y - sin(-pitch_raw)*cos(-roll_raw)*a_z
        self.ay = cos(-roll_raw)*a_y + sin(-roll_raw)*a_z

        self.prev_time = self.curr_time

    def saveHistory(self):
        """ Save measurement data into history array"""

        
        self.yaw_his.append(self.yaw)
        self.psiDot_his.append(self.psiDot)
        self.ax_his.append(self.ax)
        self.ay_his.append(self.ay)
        self.roll_his.append(self.roll)
        self.pitch_his.append(self.pitch)



class GpsClass(object):
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """

        self.x      = 0.0
        self.y      = 0.0
        
        # GPS measurement history
        self.x_his  = np.array([])
        self.y_his  = np.array([])
        
        # time stamp
        self.t0         = t0
        self.time_his   = np.array([])

    def gps_callback(self,data):
        """Unpack message from sensor, GPS"""

        self.x = data.x_m
        self.y = data.y_m

        # 1) x(t) ~ c0x + c1x * t + c2x * t^2
        # 2) y(t) ~ c0y + c1y * t + c2y * t^2
        # c_X = [c0x c1x c2x] and c_Y = [c0y c1y c2y] 
        # n_intplt = 20
        # if size(self.x_his,0) > n_intplt: # do interpolation when there is enough points
        #     x_intplt = self.x_his[-n_intplt:]
        #     y_intplt = self.y_his[-n_intplt:]
        #     t_intplt = self.time_his[-n_intplt:]
        #     t_matrix = vstack([t_intplt**2, t_intplt, ones(sz)]).T
        #     self.c_X = linalg.lstsq(t_matrix, x_intplt)[0]
        #     self.c_Y = linalg.lstsq(t_matrix, y_intplt)[0]
        #     self.x = polyval(self.c_X, self.curr_time)
        #     self.y = polyval(self.c_Y, self.curr_time)

    def saveHistory(self):

        self.x_his = np.append(self.x_his,self.x)
        self.y_his = np.append(self.y_his,self.y)


class EncClass(object):
    """ Object collecting ENC measurement data
    Attributes:
        Measurement:
            1.v_fl 2.v_fr 3. v_rl 4. v_rr
        Measurement history:
            1.v_fl_his 2.v_fr_his 3. v_rl_his 4. v_rr_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    """
    def __init__(self,t0):

        # ENC measurement
        self.v_fl      = 0.0
        self.v_fr      = 0.0
        self.v_rl      = 0.0
        self.v_rr      = 0.0
        self.v_meas    = 0.0
        
        # ENC measurement history
        self.v_fl_his    = []
        self.v_fr_his    = []
        self.v_rl_his    = []
        self.v_rr_his    = []
        self.v_meas_his  = []
        
        # time stamp
        self.v_count    = 0
        self.v_prev     = 0.0
        self.t0         = t0
        self.time_his   = []

    def enc_callback(self,data):
        """Unpack message from sensor, ENC"""

        self.v_fl = data.vel_fl
        self.v_fr = data.vel_fr
        self.v_rl = data.vel_bl
        self.v_rr = data.vel_br
        v_est = self.v_rr
        if v_est != self.v_prev:
            self.v_meas = v_est
            self.v_prev = v_est
            self.v_count = 0
        else:
            self.v_count += 1
            if self.v_count > 10:     # if 10 times in a row the same measurement
                self.v_meas = 0       # set velocity measurement to zero

    def saveHistory(self):
        
        self.v_fl_his.append(self.v_fl)
        self.v_fr_his.append(self.v_fr)
        self.v_rl_his.append(self.v_rl)
        self.v_rr_his.append(self.v_rr)

        self.v_meas_his.append(self.v_meas)

class EcuClass(object):
    """ Object collecting CMD command data
    Attributes:
        Input command:
            1.a 2.df
        Input command history:
            1.a_his 2.df_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """

        # ECU measurement
        self.a  = 0.0
        self.df = 0.0
        
        # ECU measurement history
        self.a_his  = []
        self.df_his = []
    
        # time stamp
        self.t0         = t0
        self.time_his   = []

    def ecu_callback(self,data):
        """Unpack message from sensor, ECU"""

        self.a  = data.motor
        self.df = data.servo

    def saveHistory(self):
        
        self.a_his.append(self.a)
        self.df_his.append(self.df)


if __name__ == '__main__':
    main()