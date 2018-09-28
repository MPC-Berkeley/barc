import sys
sys.path.append(sys.path[0]+'/../ControllersObject')
sys.path.append(sys.path[0]+'/../Utilities')
from dataStructures import LMPCprediction, EstimatorData, ClosedLoopDataObj, LMPCprediction

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
from LMPC import ControllerLMPC
from ZeroStepLMPC import ControllerZeroStepLMPC

import sys
import pickle
import pdb
from trackInitialization import Map
from dataStructures import LMPCprediction, EstimatorData, ClosedLoopDataObj
import os
import datetime
import scipy.io as sio

from numpy import linalg as la


def main():
    homedir = os.path.expanduser("~")
    print "Do you wanna use LMPC.obj? [y/n]"
    inputKeyBoard = raw_input()
    fromLMPC = inputKeyBoard == "y"

    if fromLMPC == True:
        file_data = open(homedir+'/barc_data/ClosedLoopDataLMPC.obj', 'rb')    
        ClosedLoopData = pickle.load(file_data)
        LMPController = pickle.load(file_data)
        LMPCOpenLoopData = pickle.load(file_data)    
    else:
        file_data = open(homedir+'/barc_data/ClosedLoopDataZeroStep.obj', 'rb')    
        ClosedLoopData = pickle.load(file_data)
        ControllerZeroStepLMPC = pickle.load(file_data)
        LMPCOpenLoopData = pickle.load(file_data)    
        LMPController = pickle.load(file_data)
    
    file_data.close()
    map = LMPController.map
    LapToPlot = range(2,4)

    plotComputationalTime(LMPController, LapToPlot, map)

    pdb.set_trace()
    print "Track length is: ", map.TrackLength

    # Plot Lap Time
    plt.figure()
    plt.plot([i*LMPController.dt for i in LMPController.LapCounter[1:LMPController.it]], '-o', label="Lap Time")
    plt.legend()
    plt.show()

    # Plot First Path Following Lap and Learning laps
    LapToPlotLearningProcess = [2, 4, 7, 15, 24]
    # LapToPlotLearningProcess = [1]
    plotClosedLoopLMPC(LMPController, map, LapToPlotLearningProcess)
    plt.show()
    
    LapToPlot = range(20,28)

    plotMeasuredAndAppliedSteering(LMPController, map, LapToPlotLearningProcess)
    plt.show()
    # Plot Best Laps
    # LapToPlot      = range(0, LMPController.it)
    BestNunberLaps = 5
    SortedTimes    = np.sort(LMPController.LapCounter[1:LMPController.it])
    LapToPlot      = np.argsort(LMPController.LapCounter[1:LMPController.it])[0:BestNunberLaps]
    # LapToPlot = range(15,19)
    LapToPlot = range(23,28)
    
    print SortedTimes
    print "Lap Plotted: ", LapToPlot, " Lap Time: ", LMPController.LapCounter[LapToPlot]
    plotClosedLoopColorLMPC(LMPController, map, LapToPlot)
    
    plotClosedLoopLMPC(LMPController, map, LapToPlot)
    # Plot Acceleration
    plotAccelerations(LMPController, LapToPlot, map)
    
    plt.show()

    # Plot One Step Prediction Error
    plotOneStepPreditionError(LMPController, LMPCOpenLoopData, LapToPlotLearningProcess)
    plt.show()

    # Computational Time    
    plotComputationalTime(LMPController, LapToPlot, map)
    plt.show()

    print "Do you wanna create xy gif? [Lap #/n]"
    inputKeyBoard = raw_input()
    if inputKeyBoard != "n":
        saveGif_xyResults(map, LMPCOpenLoopData, LMPController, int(inputKeyBoard))

    print "Do you wanna create state gif? [Lap #/n]"
    inputKeyBoard = raw_input()
    if inputKeyBoard != "n":
        Save_statesAnimation(map, LMPCOpenLoopData, LMPController, int(inputKeyBoard))
    # pdb.set_trace()
    # animation_states(map, LMPCOpenLoopData, LMPController, 10)

    print "Do you wanna create xy gif for sys ID? [Lap #/n]"
    inputKeyBoard = raw_input()
    if inputKeyBoard != "n":
        saveGif_xyResultsSysID(map, LMPCOpenLoopData, LMPController, int(inputKeyBoard))

    # plt.show()
    
def plotAccelerations(LMPController, LapToPlot, map):
    n = LMPController.n
    x = np.zeros((10000, 1, LMPController.it+2))
    s = np.zeros((10000, 1, LMPController.it+2))
    y = np.zeros((10000, 1, LMPController.it+2))
    ax = np.zeros((10000, 1, LMPController.it+2))
    ay = np.zeros((10000, 1, LMPController.it+2))
    psiDot = np.zeros((10000, 1, LMPController.it+2))
    roll = np.zeros((10000, 1, LMPController.it+2))
    pitch = np.zeros((10000, 1, LMPController.it+2))
    LapCounter = np.zeros(LMPController.it+2).astype(int)

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
    roll_his            = npz_output["roll_his"]
    pitch_his           = npz_output["pitch_his"]
    wx_his              = npz_output["wx_his"]
    wy_his              = npz_output["wy_his"]
    wz_his              = npz_output["wz_his"]
    v_rl_his            = npz_output["v_rl_his"]
    v_rr_his            = npz_output["v_rr_his"]
    v_fl_his            = npz_output["v_fl_his"]
    v_fr_his            = npz_output["v_fr_his"]
    yaw_his             = npz_output["psi_raw_his"]

    halfTrack = 0
    iteration = 0
    TimeCounter = 0
    for i in range(0, len(x_est_his)):
        s_i, ey_i, epsi_i, _ = map.getLocalPosition(x_est_his[i], y_est_his[i], yaw_est_his[i])
        
        if s_i > map.TrackLength/4 and s_i < map.TrackLength/4*3:
            halfTrack = 1
        
        if s_i < map.TrackLength/4 and halfTrack == 1:
            print "Finishced unpacking iteration: ", iteration
            halfTrack = 0
            iteration += 1
            LapCounter[iteration-1] = TimeCounter - 1
            LapCounter[iteration] = 0
            TimeCounter = 0

        if iteration > LMPController.it:
            break

        s[TimeCounter, 0, iteration]      = s_i
        ax[TimeCounter, 0, iteration]     = inp_ax_his[i]
        ay[TimeCounter, 0, iteration]     = inp_ay_his[i]
        psiDot[TimeCounter, 0, iteration] = inp_psiDot_his[i]
        roll[TimeCounter, 0, iteration]   = roll_his[i]
        pitch[TimeCounter, 0, iteration]  = pitch_his[i]

        TimeCounter += 1


    plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

    plt.figure()
    plt.subplot(511)
    counter = 0
    for i in LapToPlot:
        plt.plot(s[0:LapCounter[i], 0, i], ax[0:LapCounter[i], 0, i], '-o', label=i, color=plotColors[counter])
        counter += 1
    plt.legend(bbox_to_anchor=(0,1.02,1,0.2), borderaxespad=0, ncol=len(LapToPlot))

    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('ax [m/s^2]')
    plt.subplot(512)
    counter = 0
    for i in LapToPlot:
        plt.plot(s[0:LapCounter[i], 0, i], ay[0:LapCounter[i], 0, i], '-o', color=plotColors[counter])
        counter += 1
    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('ay [m/s^2]')
    plt.subplot(513)
    counter = 0
    for i in LapToPlot:
        plt.plot(s[0:LapCounter[i], 0, i], psiDot[0:LapCounter[i], 0, i], '-o', color=plotColors[counter])
        counter += 1
    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('psiDot [rad/s]')
    
    plt.subplot(514)
    counter = 0
    for i in LapToPlot:
        plt.plot(s[0:LapCounter[i], 0, i], pitch[0:LapCounter[i], 0, i], '-o', color=plotColors[counter])
        counter += 1
    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('psiDot [rad/s]')

    plt.subplot(515)
    counter = 0
    for i in LapToPlot:
        plt.plot(s[0:LapCounter[i], 0, i], roll[0:LapCounter[i], 0, i], '-o', color=plotColors[counter])
        counter += 1
    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('psiDot [rad/s]')
    
    

def plotComputationalTime(LMPController, LapToPlot, map):
    SS_glob = LMPController.SS_glob
    LapCounter  = LMPController.LapCounter
    SS      = LMPController.SS
    uSS     = LMPController.uSS
    qpTime  = LMPController.qpTime
    sysIDTime  = LMPController.sysIDTime
    contrTime  = LMPController.contrTime


    plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

    plt.figure()
    plt.subplot(311)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:LapCounter[i], 4, i], qpTime[0:LapCounter[i], i], '-o', label=i, color=plotColors[counter])
        counter += 1
    plt.legend(bbox_to_anchor=(0,1.02,1,0.2), borderaxespad=0, ncol=len(LapToPlot))

    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('QP solver time [s]')
    plt.subplot(312)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:LapCounter[i], 4, i], sysIDTime[0:LapCounter[i], i], '-o', color=plotColors[counter])
        counter += 1
    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('Sys ID time [s]')
    plt.subplot(313)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:LapCounter[i], 4, i], qpTime[0:LapCounter[i], i] + sysIDTime[0:LapCounter[i], i], '-o', color=plotColors[counter])
        plt.plot(SS[0:LapCounter[i], 4, i], contrTime[0:LapCounter[i], i], '-*', color=plotColors[counter])
        counter += 1
    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('Total [s]')


def plotOneStepPreditionError(LMPController, LMPCOpenLoopData, LapToPlot):
    LapCounter  = LMPController.LapCounter
    SS      = LMPController.SS
    uSS     = LMPController.uSS
    TotNumberIt = LMPController.it
    oneStepPredictionError = LMPCOpenLoopData.oneStepPredictionError

    plotColors = ['b','g','r','c','y','k','m']
    
    plt.figure(10)
    plt.subplot(611)
    plt.title("One Step Prediction Error")
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[0, 1:LapCounter[i], i], '-o', color=plotColors[counter], label=i)
        counter += 1
        plt.legend()
    plt.ylabel('vx [m/s]')
    plt.subplot(612)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[1, 1:LapCounter[i], i], '-o', color=plotColors[counter])
        counter += 1
    plt.ylabel('vy [m/s]')
    plt.subplot(613)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[2, 1:LapCounter[i], i], '-o', color=plotColors[counter])
        counter += 1
    plt.ylabel('wz [rad/s]')
    plt.subplot(614)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[3, 1:LapCounter[i], i], '-o', color=plotColors[counter])
        counter += 1
    plt.ylabel('epsi [rad]')
    plt.subplot(615)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[4, 1:LapCounter[i], i], '-o', color=plotColors[counter])
        counter += 1
    plt.ylabel('s [m]')
    plt.subplot(616)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[5, 1:LapCounter[i], i], '-o', color=plotColors[counter])
        counter += 1
    plt.ylabel('ey [m]')
    plt.xlabel('s [m]')



def plotTrajectory(map, ClosedLoop):
    x = ClosedLoop.x
    x_glob = ClosedLoop.x_glob
    u = ClosedLoop.u
    
    Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
    Points1 = np.zeros((int(Points), 2))
    Points2 = np.zeros((int(Points), 2))
    Points0 = np.zeros((int(Points), 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    plt.figure()
    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    plt.plot(x_glob[:, 4], x_glob[:, 5], '-r')

    plt.figure()
    plt.subplot(711)
    plt.plot(x[:, 4], x[:, 0], '-o')
    plt.ylabel('vx')
    plt.subplot(712)
    plt.plot(x[:, 4], x[:, 1], '-o')
    plt.ylabel('vy')
    plt.subplot(713)
    plt.plot(x[:, 4], x[:, 2], '-o')
    plt.ylabel('wz')
    plt.subplot(714)
    plt.plot(x[:, 4], x[:, 3], '-o')
    plt.ylabel('epsi')
    plt.subplot(715)
    plt.plot(x[:, 4], x[:, 5], '-o')
    plt.ylabel('ey')
    plt.subplot(716)
    plt.plot(x[0:-1, 4], u[:, 0], '-o')
    plt.ylabel('steering')
    plt.subplot(717)
    plt.plot(x[0:-1, 4], u[:, 1], '-o')
    plt.ylabel('acc')
    plt.show()


def plotClosedLoopColorLMPC(LMPController, map, LapToPlot):
    SS_glob = LMPController.SS_glob
    LapCounter  = LMPController.LapCounter
    SS      = LMPController.SS
    uSS     = LMPController.uSS

    TotNumberIt = LMPController.it

    print "Number iterations: ", TotNumberIt
    Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
    Points1 = np.zeros((int(Points), 2))
    Points2 = np.zeros((int(Points), 2))
    Points0 = np.zeros((int(Points), 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    plt.figure()
    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')

    xPlot = []
    yPlot = []
    Color = []
    for i in LapToPlot:
        for j in range(0, len(SS_glob[0:LapCounter[i], 4, i].tolist())):
            xPlot.append(SS_glob[0:LapCounter[i], 4, i].tolist()[j])
            yPlot.append(SS_glob[0:LapCounter[i], 5, i].tolist()[j])
            Color.append(np.sqrt( (SS_glob[0:LapCounter[i], 0, i].tolist()[j])**2 +  (SS_glob[0:LapCounter[i], 0, i].tolist()[j]) ) )

    plt.scatter(xPlot, yPlot, alpha=1.0, c = Color, s = 100)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    # plt.scatter(SS_glob[0:LapCounter[i], 4, i], SS_glob[0:LapCounter[i], 5, i], alpha=0.8, c = SS_glob[0:LapCounter[i], 0, i])
    plt.colorbar()


def plotClosedLoopLMPC(LMPController, map, LapToPlot):
    SS_glob = LMPController.SS_glob
    LapCounter  = LMPController.LapCounter
    SS      = LMPController.SS
    uSS     = LMPController.uSS

    plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

    TotNumberIt = LMPController.it
    print "Number iterations: ", TotNumberIt
    Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
    Points1 = np.zeros((int(Points), 2))
    Points2 = np.zeros((int(Points), 2))
    Points0 = np.zeros((int(Points), 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    plt.figure()
    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')

    plt.show()

    counter = 0
    for i in LapToPlot:
        plt.plot(SS_glob[0:LapCounter[i], 4, i], SS_glob[0:LapCounter[i], 5, i], '-o', color=plotColors[counter], label=i)
        counter += 1
    plt.legend()
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    i = 1
    sio.savemat('States.mat', {'globalStates':SS_glob[0:LapCounter[i], :, i], 'localStates':SS[0:LapCounter[i], :, i]})
    sio.savemat('Input.mat', {'input':uSS[0:LapCounter[i], :, i]})


    plt.figure()
    plt.subplot(711)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 0, i], '-o', label=i, color=plotColors[counter])
        counter += 1
    plt.legend(bbox_to_anchor=(0,1.02,1,0.2), borderaxespad=0, ncol=len(LapToPlot))

    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('vx [m/s]')
    plt.subplot(712)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 1, i], '-o', color=plotColors[counter])
        counter += 1
    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('vy [m/s]')
    plt.subplot(713)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 2, i], '-o', color=plotColors[counter])
        counter += 1
    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('wz [rad/s]')
    plt.subplot(714)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 3, i], '-o', color=plotColors[counter])
        counter += 1
    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('epsi [rad]')
    plt.subplot(715)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 5, i], '-o', color=plotColors[counter])
        counter += 1
    plt.axvline(map.TrackLength, linewidth=4, color='g')
    plt.ylabel('ey [m]')
    plt.subplot(716)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:LapCounter[i]-1, 4, i], uSS[0:LapCounter[i] - 1, 0, i], '-o', color=plotColors[counter])
        counter += 1
    plt.ylabel('Steering [rad]')
    plt.subplot(717)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:LapCounter[i]-1, 4, i], uSS[0:LapCounter[i] - 1, 1, i], '-o', color=plotColors[counter])
        counter += 1
    plt.ylabel('Acc [m/s^2]')
    plt.xlabel('s [m]')

def plotMeasuredAndAppliedSteering(LMPController, map, LapToPlot):
    plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

    SS_glob = LMPController.SS_glob
    LapCounter  = LMPController.LapCounter
    SS      = LMPController.SS
    uSS     = LMPController.uSS

    plt.figure()
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:LapCounter[i]-1, 4, i], uSS[0:LapCounter[i] - 1, 0, i], '-o', color=plotColors[counter], label="commanded Steering")
        plt.plot(SS[0:LapCounter[i]-1, 4, i], LMPController.measSteering[0:LapCounter[i] - 1, 0, i], '--*', color=plotColors[counter], label="meausred Steering")
        counter += 1
    plt.legend()
    plt.ylabel('Steering [rad]')


def animation_xy(map, LMPCOpenLoopData, LMPController, it):
    SS_glob = LMPController.SS_glob
    LapCounter = LMPController.LapCounter
    SS = LMPController.SS
    uSS = LMPController.uSS

    Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
    Points1 = np.zeros((Points, 2))
    Points2 = np.zeros((Points, 2))
    Points0 = np.zeros((Points, 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    plt.figure()
    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    plt.plot(SS_glob[0:LapCounter[it], 4, it], SS_glob[0:LapCounter[it], 5, it], '-ok', label="Closed-loop trajectory",zorder=-1)

    ax = plt.axes()
    SSpoints_x = []; SSpoints_y = []
    xPred = []; yPred = []
    SSpoints, = ax.plot(SSpoints_x, SSpoints_y, 'sb', label="SS",zorder=0)
    line, = ax.plot(xPred, yPred, '-or', label="Predicted Trajectory",zorder=1)

    v = np.array([[ 1.,  1.],
                  [ 1., -1.],
                  [-1., -1.],
                  [-1.,  1.]])
    rec = patches.Polygon(v, alpha=0.7,closed=True, fc='r', ec='k',zorder=10)
    ax.add_patch(rec)

    plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3)

    N = LMPController.N
    numSS_Points = LMPController.numSS_Points
    for i in range(0, int(LMPController.LapCounter[it])):

        xPred = np.zeros((N+1, 1)); yPred = np.zeros((N+1, 1))
        SSpoints_x = np.zeros((numSS_Points, 1)); SSpoints_y = np.zeros((numSS_Points, 1))

        for j in range(0, N+1):
            xPred[j,0], yPred[j,0]  = map.getGlobalPosition( LMPCOpenLoopData.PredictedStates[j, 4, i, it],
                                                             LMPCOpenLoopData.PredictedStates[j, 5, i, it] )

            if j == 0:
                x = SS_glob[i, 4, it]
                y = SS_glob[i, 5, it]
                psi = SS_glob[i, 3, it]
                l = 0.4; w = 0.2
                car_x = [ x + l * np.cos(psi) - w * np.sin(psi), x + l*np.cos(psi) + w * np.sin(psi),
                          x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
                car_y = [ y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
                          y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]




        for j in range(0, numSS_Points):
            SSpoints_x[j,0], SSpoints_y[j,0] = map.getGlobalPosition(LMPCOpenLoopData.SSused[4, j, i, it],
                                                                     LMPCOpenLoopData.SSused[5, j, i, it])
        SSpoints.set_data(SSpoints_x, SSpoints_y)

        line.set_data(xPred, yPred)

        rec.set_xy(np.array([car_x, car_y]).T)

        plt.draw()
        plt.pause(1e-17)

def animation_states(map, LMPCOpenLoopData, LMPController, it):
    SS_glob = LMPController.SS_glob
    LapCounter = LMPController.LapCounter
    SS = LMPController.SS
    uSS = LMPController.uSS

    xdata = []; ydata = []
    fig = plt.figure()

    axvx = fig.add_subplot(3, 2, 1)
    plt.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 0, it], '-ok', label="Closed-loop trajectory")
    lineSSvx, = axvx.plot(xdata, ydata, 'sb-', label="SS")
    linevx, = axvx.plot(xdata, ydata, 'or-', label="Predicted Trajectory")
    plt.ylabel("vx")
    plt.xlabel("s")

    plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3)

    axvy = fig.add_subplot(3, 2, 2)
    axvy.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 1, it], '-ok')
    lineSSvy, = axvy.plot(xdata, ydata, 'sb-')
    linevy, = axvy.plot(xdata, ydata, 'or-')
    plt.ylabel("vy")
    plt.xlabel("s")

    axwz = fig.add_subplot(3, 2, 3)
    axwz.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 2, it], '-ok')
    lineSSwz, = axwz.plot(xdata, ydata, 'sb-')
    linewz, = axwz.plot(xdata, ydata, 'or-')
    plt.ylabel("wz")
    plt.xlabel("s")

    axepsi = fig.add_subplot(3, 2, 4)
    axepsi.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 3, it], '-ok')
    lineSSepsi, = axepsi.plot(xdata, ydata, 'sb-')
    lineepsi, = axepsi.plot(xdata, ydata, 'or-')
    plt.ylabel("epsi")
    plt.xlabel("s")

    axey = fig.add_subplot(3, 2, 5)
    axey.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 5, it], '-ok')
    lineSSey, = axey.plot(xdata, ydata, 'sb-')
    lineey, = axey.plot(xdata, ydata, 'or-')
    plt.ylabel("ey")
    plt.xlabel("s")

    Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
    Points1 = np.zeros((Points, 2))
    Points2 = np.zeros((Points, 2))
    Points0 = np.zeros((Points, 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    axtr = fig.add_subplot(3, 2, 6)
    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    plt.plot(SS_glob[0:LapCounter[it], 4, it], SS_glob[0:LapCounter[it], 5, it], '-ok')

    SSpoints_x = []; SSpoints_y = []
    xPred = []; yPred = []
    SSpoints_tr, = axtr.plot(SSpoints_x, SSpoints_y, 'sb')
    line_tr, = axtr.plot(xPred, yPred, '-or')

    N = LMPController.N
    numSS_Points = LMPController.numSS_Points
    for i in range(0, int(LMPController.LapCounter[it])):

        xPred    = LMPCOpenLoopData.PredictedStates[:, :, i, it]
        SSpoints = LMPCOpenLoopData.SSused[:, :, i, it]

        linevx.set_data(xPred[:, 4], xPred[:, 0])
        linevy.set_data(xPred[:, 4], xPred[:, 1])
        linewz.set_data(xPred[:, 4], xPred[:, 2])
        lineepsi.set_data(xPred[:, 4], xPred[:, 3])
        lineey.set_data(xPred[:, 4], xPred[:, 5])

        lineSSvx.set_data(SSpoints[4,:], SSpoints[0,:])
        lineSSvy.set_data(SSpoints[4,:], SSpoints[1,:])
        lineSSwz.set_data(SSpoints[4,:], SSpoints[2,:])
        lineSSepsi.set_data(SSpoints[4,:], SSpoints[3,:])
        lineSSey.set_data(SSpoints[4,:], SSpoints[5,:])

        xPred = np.zeros((N + 1, 1));yPred = np.zeros((N + 1, 1))
        SSpoints_x = np.zeros((numSS_Points, 1));SSpoints_y = np.zeros((numSS_Points, 1))

        for j in range(0, N + 1):
            xPred[j, 0], yPred[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.PredictedStates[j, 4, i, it],
                                                             LMPCOpenLoopData.PredictedStates[j, 5, i, it])

        for j in range(0, numSS_Points):
            SSpoints_x[j, 0], SSpoints_y[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.SSused[4, j, i, it],
                                                                       LMPCOpenLoopData.SSused[5, j, i, it])

        line_tr.set_data(xPred, yPred)
        SSpoints_tr.set_data(SSpoints_x, SSpoints_y)

        plt.draw()
        plt.pause(1e-17)

def saveGif_xyResults(map, LMPCOpenLoopData, LMPController, it):
    SS_glob = LMPController.SS_glob
    LapCounter = LMPController.LapCounter
    SS = LMPController.SS
    uSS = LMPController.uSS

    Points = int(np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4])))
    Points1 = np.zeros((Points, 2))
    Points2 = np.zeros((Points, 2))
    Points0 = np.zeros((Points, 2))
    for i in range(0, Points):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    pdb.set_trace()
    fig = plt.figure()
    # plt.ylim((-5, 1.5))
    fig.set_tight_layout(True)
    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    plt.plot(SS_glob[0:LapCounter[it], 4, it], SS_glob[0:LapCounter[it], 5, it], '-ok', label="Closed-loop trajectory", markersize=1,zorder=-1)

    ax = plt.axes()
    SSpoints_x = []; SSpoints_y = []
    xPred = []; yPred = []
    SSpoints, = ax.plot(SSpoints_x, SSpoints_y, 'sb', label="SS",zorder=0)
    line, = ax.plot(xPred, yPred, '-or', label="Predicted Trajectory",zorder=1)

    v = np.array([[ 1.,  1.],
                  [ 1., -1.],
                  [-1., -1.],
                  [-1.,  1.]])
    rec = patches.Polygon(v, alpha=0.7,closed=True, fc='r', ec='k',zorder=10)
    ax.add_patch(rec)

    plt.legend(mode="expand", ncol=3)
    # plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
    #             mode="expand", borderaxespad=0, ncol=3)

    N = LMPController.N
    numSS_Points = LMPController.numSS_Points

    def update(i):
        xPred = np.zeros((N + 1, 1)); yPred = np.zeros((N + 1, 1))
        SSpoints_x = np.zeros((numSS_Points, 1)); SSpoints_y = np.zeros((numSS_Points, 1))

        for j in range(0, N + 1):
            xPred[j, 0], yPred[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.PredictedStates[j, 4, i, it],
                                                             LMPCOpenLoopData.PredictedStates[j, 5, i, it])

            if j == 0:
                x = SS_glob[i, 4, it]
                y = SS_glob[i, 5, it]
                psi = SS_glob[i, 3, it]
                l = 0.4;w = 0.2
                car_x = [x + l * np.cos(psi) - w * np.sin(psi), x + l * np.cos(psi) + w * np.sin(psi),
                         x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
                car_y = [y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
                         y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]

        for j in range(0, numSS_Points):
            SSpoints_x[j, 0], SSpoints_y[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.SSused[4, j, i, it],
                                                                       LMPCOpenLoopData.SSused[5, j, i, it])
        SSpoints.set_data(SSpoints_x, SSpoints_y)

        line.set_data(xPred, yPred)

        rec.set_xy(np.array([car_x, car_y]).T)

    anim = FuncAnimation(fig, update, frames=np.arange(0, int(LMPController.LapCounter[it])), interval=100)

    anim.save('gif/ClosedLoop/ClosedLoop.gif', dpi=80, writer='imagemagick')

def saveGif_xyResultsSysID(map, LMPCOpenLoopData, LMPController, it):
    SS_glob = LMPController.SS_glob
    LapCounter = LMPController.LapCounter
    SS = LMPController.SS
    uSS = LMPController.uSS

    Points = int(np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4])))
    Points1 = np.zeros((Points, 2))
    Points2 = np.zeros((Points, 2))
    Points0 = np.zeros((Points, 2))
    for i in range(0, Points):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    pdb.set_trace()
    fig = plt.figure()
    # plt.ylim((-5, 1.5))
    fig.set_tight_layout(True)
    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    plt.plot(SS_glob[0:LapCounter[it], 4, it], SS_glob[0:LapCounter[it], 5, it], '-ok', label="Closed-loop trajectory", markersize=1,zorder=-1)

    ax = plt.axes()
    SSpoints_x = []; SSpoints_y = []
    xPred = []; yPred = []
    SSpoints, = ax.plot(SSpoints_x, SSpoints_y, 'sb', label="Used Data",zorder=0)
    line, = ax.plot(xPred, yPred, '-or',zorder=1)

    v = np.array([[ 1.,  1.],
                  [ 1., -1.],
                  [-1., -1.],
                  [-1.,  1.]])
    rec = patches.Polygon(v, alpha=0.7,closed=True, fc='r', ec='k',zorder=10)
    ax.add_patch(rec)

    plt.legend(mode="expand", ncol=3)
    # plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
    #             mode="expand", borderaxespad=0, ncol=3)

    N = LMPController.N
    numSS_Points = LMPController.numSS_Points

    def update(i):
        xPred = np.zeros((N + 1, 1)); 
        yPred = np.zeros((N + 1, 1))

        scaling = np.array([[0.1, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 1.0]])

        xLin = np.hstack((SS[i, [0, 1, 2], it], uSS[i, :, it]))
        h = 2 * 5
        lamb = 0.0
        stateFeatures = [0, 1, 2]
        usedIt = [it - 2, it -1]

        indexSelected = []
        K = []
        LapCounter = LMPController.LapCounter
        MaxNumPoint= LMPController.MaxNumPoint
        print xLin
        for iterationUsed in usedIt:
            indexSelected_i, K_i = ComputeIndex(h, SS, uSS, LapCounter, iterationUsed, xLin, stateFeatures, scaling, MaxNumPoint,1, 0, 0)
            indexSelected.append(indexSelected_i)
            K.append(K_i)

        Counter = 0

        # Compute Matrices For Local Linear Regression
        stateFeatures = [4, 5]
        DataSysID   = np.empty((0,len(stateFeatures)))

        for indexIterations in usedIt:
            DataSysID = np.append( DataSysID, np.squeeze(SS_glob[np.ix_(indexSelected[Counter], stateFeatures, [indexIterations])]), axis=0)
            Counter = Counter + 1

        # pdb.set_trace()

        # SSpoints_x = np.zeros((numSS_Points, 1)); SSpoints_y = np.zeros((numSS_Points, 1))

        # for j in range(0, N + 1):
        #     xPred[j, 0], yPred[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.PredictedStates[j, 4, i, it],
        #                                                      LMPCOpenLoopData.PredictedStates[j, 5, i, it])

        #     if j == 0:
        #         x = SS_glob[i, 4, it]
        #         y = SS_glob[i, 5, it]
        #         psi = SS_glob[i, 3, it]
        #         l = 0.4;w = 0.2
        #         car_x = [x + l * np.cos(psi) - w * np.sin(psi), x + l * np.cos(psi) + w * np.sin(psi),
        #                  x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
        #         car_y = [y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
        #                  y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]

        # for j in range(0, numSS_Points):
        #     SSpoints_x[j, 0], SSpoints_y[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.SSused[4, j, i, it],
        #                                                                LMPCOpenLoopData.SSused[5, j, i, it])
        SSpoints.set_data(DataSysID[:,0], DataSysID[:,1])

        x = SS_glob[i, 4, it]
        y = SS_glob[i, 5, it]
        psi = SS_glob[i, 3, it]
        l = 2*0.15;
        w = 2*0.075
        car_x = [x + l * np.cos(psi) - w * np.sin(psi), x + l * np.cos(psi) + w * np.sin(psi),
                 x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
        car_y = [y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
                 y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]
        rec.set_xy(np.array([car_x, car_y]).T)

    anim = FuncAnimation(fig, update, frames=np.arange(0, int(LMPController.LapCounter[it])), interval=100)

    anim.save('gif/ClosedLoop/ClosedLoop.gif', dpi=80, writer='imagemagick')



def ComputeIndex(h, SS, uSS, LapCounter, it, x0, stateFeatures, scaling, MaxNumPoint, ConsiderInput, steeringDelay, idDelay):
    startTimer = datetime.datetime.now()  # Start timer for LMPC iteration

    # What to learn a model such that: x_{k+1} = A x_k  + B u_k + C
    startTime = 0
    endTime   = LapCounter[it] - 1

    oneVec = np.ones( (SS[startTime:endTime, :, it].shape[0], 1) )

    x0Vec = (np.dot( np.array([x0]).T, oneVec.T )).T

    if ConsiderInput == 1:
        DataMatrix = np.hstack((SS[startTime:endTime, stateFeatures, it], uSS[startTime:endTime, :, it]))
    else:
        DataMatrix = SS[startTime:endTime, stateFeatures, it]

    diff  = np.dot(( DataMatrix - x0Vec ), scaling)
    # print 'x0Vec \n',x0Vec
    norm = la.norm(diff, 1, axis=1)
    
    # Need to make sure that the indices [0:steeringDelay] are not selected as it us needed to shift the input vector
    if (steeringDelay+idDelay) > 0:
        norm[0:(steeringDelay+idDelay)] = 10000

    indexTot =  np.squeeze(np.where(norm < h))
    # print indexTot.shape, np.argmin(norm), norm, x0
    if (indexTot.shape[0] >= MaxNumPoint):
        index = np.argsort(norm)[0:MaxNumPoint]
        # MinNorm = np.argmin(norm)
        # if MinNorm+MaxNumPoint >= indexTot.shape[0]:
        #     index = indexTot[indexTot.shape[0]-MaxNumPoint:indexTot.shape[0]]
        # else:
        #     index = indexTot[MinNorm:MinNorm+MaxNumPoint]
    else:
        index = indexTot

    K  = ( 1 - ( norm[index] / h )**2 ) * 3/4
    # K = np.ones(len(index))

    return index, K

def Save_statesAnimation(map, LMPCOpenLoopData, LMPController, it):
    SS_glob = LMPController.SS_glob
    LapCounter = LMPController.LapCounter
    SS = LMPController.SS
    uSS = LMPController.uSS

    xdata = []; ydata = []
    fig = plt.figure()
    fig.set_tight_layout(True)

    axvx = fig.add_subplot(3, 2, 1)
    plt.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 0, it], '-ok', label="Closed-loop trajectory")
    lineSSvx, = axvx.plot(xdata, ydata, 'sb-', label="SS")
    linevx, = axvx.plot(xdata, ydata, 'or-', label="Predicted Trajectory")
    plt.ylabel("vx")
    plt.xlabel("s")

    plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3)

    axvy = fig.add_subplot(3, 2, 2)
    axvy.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 1, it], '-ok')
    lineSSvy, = axvy.plot(xdata, ydata, 'sb-')
    linevy, = axvy.plot(xdata, ydata, 'or-')
    plt.ylabel("vy")
    plt.xlabel("s")

    axwz = fig.add_subplot(3, 2, 3)
    axwz.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 2, it], '-ok')
    lineSSwz, = axwz.plot(xdata, ydata, 'sb-')
    linewz, = axwz.plot(xdata, ydata, 'or-')
    plt.ylabel("wz")
    plt.xlabel("s")

    axepsi = fig.add_subplot(3, 2, 4)
    axepsi.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 3, it], '-ok')
    lineSSepsi, = axepsi.plot(xdata, ydata, 'sb-')
    lineepsi, = axepsi.plot(xdata, ydata, 'or-')
    plt.ylabel("epsi")
    plt.xlabel("s")

    axey = fig.add_subplot(3, 2, 5)
    axey.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 5, it], '-ok')
    lineSSey, = axey.plot(xdata, ydata, 'sb-')
    lineey, = axey.plot(xdata, ydata, 'or-')
    plt.ylabel("ey")
    plt.xlabel("s")

    Points = int(np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4])))
    Points1 = np.zeros((Points, 2))
    Points2 = np.zeros((Points, 2))
    Points0 = np.zeros((Points, 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    axtr = fig.add_subplot(3, 2, 6)
    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    plt.plot(SS_glob[0:LapCounter[it], 4, it], SS_glob[0:LapCounter[it], 5, it], '-ok')

    SSpoints_x = []; SSpoints_y = []
    xPred = []; yPred = []
    SSpoints_tr, = axtr.plot(SSpoints_x, SSpoints_y, 'sb')
    line_tr, = axtr.plot(xPred, yPred, '-or')

    N = LMPController.N
    numSS_Points = LMPController.numSS_Points

    def update(i):
        xPred    = LMPCOpenLoopData.PredictedStates[:, :, i, it]
        SSpoints = LMPCOpenLoopData.SSused[:, :, i, it]

        linevx.set_data(xPred[:, 4], xPred[:, 0])
        linevy.set_data(xPred[:, 4], xPred[:, 1])
        linewz.set_data(xPred[:, 4], xPred[:, 2])
        lineepsi.set_data(xPred[:, 4], xPred[:, 3])
        lineey.set_data(xPred[:, 4], xPred[:, 5])

        lineSSvx.set_data(SSpoints[4,:], SSpoints[0,:])
        lineSSvy.set_data(SSpoints[4,:], SSpoints[1,:])
        lineSSwz.set_data(SSpoints[4,:], SSpoints[2,:])
        lineSSepsi.set_data(SSpoints[4,:], SSpoints[3,:])
        lineSSey.set_data(SSpoints[4,:], SSpoints[5,:])

        xPred = np.zeros((N + 1, 1));yPred = np.zeros((N + 1, 1))
        SSpoints_x = np.zeros((numSS_Points, 1));SSpoints_y = np.zeros((numSS_Points, 1))

        for j in range(0, N + 1):
            xPred[j, 0], yPred[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.PredictedStates[j, 4, i, it],
                                                             LMPCOpenLoopData.PredictedStates[j, 5, i, it])

        for j in range(0, numSS_Points):
            SSpoints_x[j, 0], SSpoints_y[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.SSused[4, j, i, it],
                                                                       LMPCOpenLoopData.SSused[5, j, i, it])

        line_tr.set_data(xPred, yPred)
        SSpoints_tr.set_data(SSpoints_x, SSpoints_y)

    anim = FuncAnimation(fig, update, frames=np.arange(0, int(LMPController.LapCounter[it])), interval=100)

    anim.save('ClosedLoopStates.gif', dpi=80, writer='imagemagick')


main()