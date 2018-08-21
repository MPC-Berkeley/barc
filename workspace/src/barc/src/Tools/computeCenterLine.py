import sys
sys.path.append(sys.path[0]+'/../RacingLMPC/ControllersObject')
sys.path.append(sys.path[0]+'/../RacingLMPC/Utilities')
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
import scipy.io as sio

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
    
    map = LMPController.map

    s = 0 
    x = []
    y = []
    x_inner = []
    y_inner = []
    x_outer = []
    y_outer = []

    while s<map.TrackLength:
        x_pos, y_pos = map.getGlobalPosition(s, 0)
        x.append(x_pos)
        y.append(y_pos)
        x_pos_inner, y_pos_inner = map.getGlobalPosition(s, -0.4)
        x_inner.append(x_pos_inner)
        y_inner.append(y_pos_inner)
        x_pos_outer, y_pos_outer = map.getGlobalPosition(s, 0.4)
        x_outer.append(x_pos_outer)
        y_outer.append(y_pos_outer)
        s = s + 0.02

    sio.savemat('x.mat', {'x':x})
    sio.savemat('y.mat', {'y':y})
    sio.savemat('CoordinatesRightFormat.mat', {'x_inner':x_inner, 'y_inner':y_inner, 'x_outer':x_outer, 'y_outer':y_outer})

    # sio.savemat('x_inner.mat', {'x_inner':x_inner})
    # sio.savemat('y_inner.mat', {'y_inner':y_inner})
    # sio.savemat('x_outer.mat', {'x_outer':x_outer})
    # sio.savemat('y_outer.mat', {'y_outer':y_outer})

    plt.figure()
    plt.plot(x,y,'o')
    plt.show()
main()