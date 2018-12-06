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
    LapToPlot = [10,15,22,27,38,39]
    plotSteeringGain(LMPController, map, LapToPlot)
    
    # Plot Acceleration
    plotAccelerations(LMPController, LapToPlot, map)
    
    LapToPlot = [10,15,22,27,30,31,32]
    plotSteeringGain(LMPController, map, LapToPlot)
    
    # Plot Acceleration
    plotAccelerations(LMPController, LapToPlot, map)
    
    plt.show()

    # plt.show()
    
def plotSteeringGain(LMPController, map, LapToPlot):
    plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

    SS_glob = LMPController.SS_glob
    LapCounter  = LMPController.LapCounter
    SS      = LMPController.SS
    uSS     = LMPController.uSS

    plt.figure()
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:(LapCounter[i]-1), 4, i], LMPController.steeringGain[0:(LapCounter[i] - 1), 0, i], '-o', color=plotColors[counter], label=i)
        counter += 1
    plt.legend()
    plt.ylabel('Steering Gain [rad]')
    plt.xlabel('s [m]')

def plotAccelerations(LMPController, LapToPlot, map):
    n = LMPController.n
    s     = np.zeros((10000, 1, LMPController.it+2))
    FyF   = np.zeros((10000, 1, LMPController.it+2))
    FyR   = np.zeros((10000, 1, LMPController.it+2))
    slipF = np.zeros((10000, 1, LMPController.it+2))
    slipR = np.zeros((10000, 1, LMPController.it+2))
    
    LapCounter = np.zeros(LMPController.it+2).astype(int)

    homedir    = os.path.expanduser("~")
    pathSave   = os.path.join(homedir,"barc_data/simulator_output.npz")
    npz_output = np.load(pathSave)
    x_his      = npz_output["x"]
    y_his      = npz_output["y"]
    yaw_his    = npz_output["yaw"]
    FyF_his    = npz_output["FyF"] 
    FyR_his    = npz_output["FyR"] 
    slipF_his  = npz_output["slipF"] 
    slipR_his  = npz_output["slipR"] 
    
    halfTrack = 0
    iteration = 0
    TimeCounter = 0

    for i in range(0, len(x_his)):
        s_i, ey_i, epsi_i, _ = map.getLocalPosition(x_his[i], y_his[i], yaw_his[i])
        
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

        s[TimeCounter, 0, iteration]     = s_i
        FyF[TimeCounter, 0, iteration]   = FyF_his[i]
        FyR[TimeCounter, 0, iteration]   = FyR_his[i]
        slipF[TimeCounter, 0, iteration] = slipF_his[i]
        slipR[TimeCounter, 0, iteration] = slipR_his[i]

        TimeCounter += 1


    plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

    plt.figure()
    counter = 0
    for i in LapToPlot:
        indexToUse = np.argsort(FyF[0:LapCounter[i]-1, 0, i])
        plt.plot(slipF[indexToUse, 0, i], FyF[indexToUse, 0, i], '-o', label=i, color=plotColors[counter])
        counter += 1
    plt.legend(bbox_to_anchor=(0,1.02,1,0.2), borderaxespad=0, ncol=len(LapToPlot))

    plt.ylabel('Lateral Force [N]')
    plt.xlabel('Slip Angle [deg]')

    plt.figure()
    counter = 0
    for i in LapToPlot:
        indexToUse = np.argsort( FyR[0:LapCounter[i]-1, 0, i])
        plt.plot(slipR[indexToUse, 0, i], FyR[indexToUse, 0, i], '-o', color=plotColors[counter])
        counter += 1
    plt.ylabel('Lateral Force [N]')
    plt.xlabel('Slip Angle [deg]')

main()