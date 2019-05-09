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
from numpy import tan, arctan, cos, sin, pi

from numpy import linalg as la

class simulator():

    def __init__(self):

        self.start_vx     = 0
        self.start_vy     = 0
        self.start_psiDot = 0
        self.start_yaw    = 0
        self.start_x      = 0
        self.start_y      = 0
        self.start_ax      = 0
        self.start_ay      = 0    
        
        self.vx     = 0
        self.vy     = 0
        self.psiDot = 0
        self.yaw    = 0
        self.x      = 0
        self.y      = 0

        self.dt = 0.01
        self.B  = 6.0 
        self.C  = 1.6 
        self.mu = 0.8 
        self.g  = 9.81

        self.L_f=0.125
        self.L_r=0.125
        self.m=1.98
        self.I_z=0.03
        self.c_f = 0.05
    
        self.ax = 0
        self.ay = 0

    def pacejka(self,ang):
        D = self.mu*self.m*self.g/2
        C_alpha_f = D*sin(self.C*arctan(self.B*ang))
        return C_alpha_f

    def f(self,u):
        a_F = 0.0
        a_R = 0.0

        if abs(self.vx) > 0.2:
            a_F = arctan((self.vy + self.L_f*self.psiDot)/abs(self.vx)) - u[1]
            a_R = arctan((self.vy - self.L_r*self.psiDot)/abs(self.vx))

        FyF = -self.pacejka(a_F)
        FyR = -self.pacejka(a_R)

        if abs(a_F) > 30.0/180.0*pi or abs(a_R) > 30.0/180.0*pi:
            print "WARNING: Large slip angles in simulation"


        x   = self.x
        y   = self.y
        vx  = self.vx
        vy  = self.vy
        yaw = self.yaw
        psiDot = self.psiDot
        ax  = self.ax
        ay  = self.ay

        self.ax      = u[0] - self.c_f*vx - FyF/self.m*sin(u[1])
        self.ay      = 1.0/self.m*(FyF*cos(u[1])+FyR)

        self.x      += self.dt*(cos(yaw)*vx - sin(yaw)*vy)
        self.y      += self.dt*(sin(yaw)*vx + cos(yaw)*vy)
        self.vx     += self.dt*(ax + psiDot*vy)
        self.vy     += self.dt*(ay - psiDot*vx)
        self.yaw    += self.dt*(psiDot)                                        
        self.psiDot += self.dt*(1.0/self.I_z*(self.L_f*FyF*cos(u[1]) - self.L_r*FyR))

        self.vx = abs(self.vx)

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
    LapToPlot = [10,18,25,32,37,38,39,40]

    LapToPlot = [40, 30, 15, 5]
    plotSteeringGain(LMPController, map, LapToPlot)
    
    # Plot Acceleration
    plotAccelerations(LMPController, LapToPlot, map)
    
    LapToPlot = [47,48,49,50]
    plotSteeringGain(LMPController, map, LapToPlot)
    
    # Plot Acceleration
    plotAccelerations(LMPController, LapToPlot, map)
    
    plt.show() 

    x = LMPController.SS_glob
    u = LMPController.uSS

    sim = simulator()
    inputKeyBoard = 'a'
    
    while (inputKeyBoard != 'k'):

        print "Pick s or kill 'k'"
        inputKeyBoard = raw_input()
        print "Pick LapToPlot"
        dummy = raw_input()
        LapToPlot = int(dummy)
        print "Lap time", LMPController.LapCounter[LapToPlot]
        # plotClosedLoopLMPC(LMPController, map, [LapToPlot])

        # Initialize State
        sim.start_vx     = x[int(inputKeyBoard), 0, LapToPlot]
        sim.start_vy     = x[int(inputKeyBoard), 1, LapToPlot]
        sim.start_psiDot = x[int(inputKeyBoard), 2, LapToPlot]
        sim.start_yaw    = x[int(inputKeyBoard), 3, LapToPlot]
        sim.start_x      = x[int(inputKeyBoard), 4, LapToPlot]
        sim.start_y      = x[int(inputKeyBoard), 5, LapToPlot]

        vx_old     = x[int(inputKeyBoard)-1, 0, LapToPlot]
        vy_old     = x[int(inputKeyBoard)-1, 1, LapToPlot]
        psiDot_old = x[int(inputKeyBoard)-1, 2, LapToPlot]
        delta_old  = u[int(inputKeyBoard)-1, 0, LapToPlot]
        acc_old    = u[int(inputKeyBoard)-1, 1, LapToPlot]

        a_F = arctan((vy_old + sim.L_f*psiDot_old)/abs(vx_old)) - delta_old
        a_R = arctan((vy_old - sim.L_r*psiDot_old)/abs(vx_old))

        FyF = -sim.pacejka(a_F)
        FyR = -sim.pacejka(a_R)

        sim.start_ax = acc_old  - sim.c_f*sim.start_vx - FyF/sim.m*sin(delta_old)
        sim.start_ay = 1.0/sim.m*(FyF*cos(delta_old)+FyR)

        acc = u[int(inputKeyBoard), 1, LapToPlot]

        # Extract matrices for prediction
        Bpred_psiDot = LMPController.steeringGain_psiDot[int(inputKeyBoard), 0, LapToPlot]
        Apred_psiDot = LMPController.steeringGain_psiDot[int(inputKeyBoard), 1:4, LapToPlot]
        Cpred_psiDot = LMPController.steeringGain_psiDot[int(inputKeyBoard), 4, LapToPlot]

        Bpred_vy = LMPController.steeringGain_vy[int(inputKeyBoard), 0, LapToPlot]
        Apred_vy = LMPController.steeringGain_vy[int(inputKeyBoard), 1:4, LapToPlot]
        Cpred_vy = LMPController.steeringGain_vy[int(inputKeyBoard), 4, LapToPlot]

        Bpred_vx = LMPController.steeringGain_vx[int(inputKeyBoard), 0, LapToPlot]
        Apred_vx = LMPController.steeringGain_vx[int(inputKeyBoard), 1:4, LapToPlot]
        Cpred_vx = LMPController.steeringGain_vx[int(inputKeyBoard), 4, LapToPlot]

        # Simulate for different steering angles
        vy_outTrue = []
        vy_outPred = []
        vy_outNext = []
        vy_outTrue_1 = []
        psiDot_outTrue = []
        psiDot_outPred = []
        delta= [u[int(inputKeyBoard), 0, LapToPlot]]
        delta_in = u[int(inputKeyBoard), 0, LapToPlot] + [-0.1,-0.075,-0.05,-0.025,-0.01,0,0.01,0.025,0.05,0.075,0.1]
        for delta in delta_in:
            # Reset Initial Condition for each delta in delta_in
            sim.vx     = sim.start_vx
            sim.vy     = sim.start_vy
            sim.psiDot = sim.start_psiDot
            sim.yaw    = sim.start_yaw
            sim.x      = sim.start_x
            sim.y      = sim.start_y

            sim.ax      = sim.start_ax
            sim.ay      = sim.start_ay

            # Simulate forward
            simulateForCostDelta(sim, delta, acc)
            vy_outTrue.append(sim.vy)            

            # Reset Initial Condition for each delta in delta_in
            sim.vx     = sim.start_vx
            sim.vy     = sim.start_vy
            sim.psiDot = sim.start_psiDot
            sim.yaw    = sim.start_yaw
            sim.x      = sim.start_x
            sim.y      = sim.start_y

            sim.ax      = sim.start_ax
            sim.ay      = sim.start_ay

            # Simulate forward
            simulateForCostDelta_1(sim, delta, acc)

            # Save Output
            vy_outTrue_1.append(sim.vy)

            # Save Output



            vx_next     = Apred_vx[0]    *sim.start_vx + Apred_vx[1]*    sim.start_vy + Apred_vx[2]*    sim.start_psiDot + Bpred_vx*acc       + Cpred_vx
            vy_next     = Apred_vy[0]    *sim.start_vx + Apred_vy[1]*    sim.start_vy + Apred_vy[2]*    sim.start_psiDot + Bpred_vy*delta     + Cpred_vy
            psiDot_next = Apred_psiDot[0]*sim.start_vx + Apred_psiDot[1]*sim.start_vy + Apred_psiDot[2]*sim.start_psiDot + Bpred_psiDot*delta + Cpred_psiDot

            vy_outPred.append(vy_next)
            vy_outNext.append(Apred_vy[0]*vx_next + Apred_vy[1]*vy_next + Apred_vy[2]*psiDot_next + Bpred_vy*delta + Cpred_vy)

            psiDot_outTrue.append(sim.psiDot)

            psiDot_outPred.append(Apred_psiDot[0]*sim.start_vx + Apred_psiDot[1]*sim.start_vy + Apred_psiDot[2]*sim.start_psiDot + Bpred_psiDot*delta + Cpred_psiDot)

        plt.figure()
        plt.plot(delta_in, psiDot_outTrue, '-o', color='r', label='psiDot_outTrue')
        plt.plot(delta_in, psiDot_outPred, '-s', color='b', label='psiDot_outPred')
        plt.ylabel('psiDot [m/s]')
        plt.xlabel('delta [rad]')
        plt.legend()

        plt.figure()
        plt.plot(delta_in, vy_outTrue_1, '-o', color='k', label='True')
        plt.plot(delta_in, vy_outNext,   '-s', color='b', label='Identified')
        plt.ylabel('vy [m/s]')
        plt.xlabel('delta [rad]')
        plt.legend()

        plt.figure()
        plt.plot(delta_in, vy_outTrue_1, '-o', color='k', label='vy_outTrue')
        plt.plot(delta_in, vy_outTrue, '-o', color='r', label='vy_outTrue')
        plt.plot(delta_in, vy_outPred, '-s', color='b', label='vy_outPred')
        plt.plot(delta_in, vy_outNext, '-s', color='g', label='vy_outPred')
        plt.ylabel('vy [m/s]')
        plt.xlabel('delta [rad]')
        plt.legend()

        print x[int(inputKeyBoard), 4, LapToPlot]
        print u[int(inputKeyBoard), 0, LapToPlot], u[int(inputKeyBoard), 1, LapToPlot] 
        print x[int(inputKeyBoard), 1, LapToPlot], x[int(inputKeyBoard)+1, 1, LapToPlot], sim.vy, vy_outPred[-1]
        print x[int(inputKeyBoard), 2, LapToPlot], x[int(inputKeyBoard)+1, 2, LapToPlot], sim.psiDot, psiDot_outPred[-1]
        
        plt.show()

        # Simulate for different vx

        vy_outTrue = []
        vy_outPred = []
        vy_outNext= []
        vy_outTrue_1 = []
        psiDot_outTrue = []
        psiDot_outPred = []
        vx_in = sim.start_vx + [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]
        for vx in vx_in:
            # Reset Initial Condition for each delta in delta_in
            sim.vx     = vx
            sim.vy     = sim.start_vy
            sim.psiDot = sim.start_psiDot
            sim.yaw    = sim.start_yaw
            sim.x      = sim.start_x
            sim.y      = sim.start_y

            sim.ax      = acc_old  - sim.c_f*vx - FyF/sim.m*sin(delta_old)
            sim.ay      = sim.start_ay

            # Simulate forward
            simulateForCostDelta(sim, delta, acc)
            vy_outTrue.append(sim.vy)

            # Reset Initial Condition for each delta in delta_in
            sim.vx     = vx
            sim.vy     = sim.start_vy
            sim.psiDot = sim.start_psiDot
            sim.yaw    = sim.start_yaw
            sim.x      = sim.start_x
            sim.y      = sim.start_y

            sim.ax      = acc_old  - sim.c_f*vx - FyF/sim.m*sin(delta_old)
            sim.ay      = sim.start_ay

            # Simulate forward
            simulateForCostDelta_1(sim, delta, acc)
            vy_outTrue_1.append(sim.vy)

            acc = u[int(inputKeyBoard), 1, LapToPlot]
            delta = u[int(inputKeyBoard), 0, LapToPlot]

            vx_next     = Apred_vx[0]    *vx + Apred_vx[1]*    sim.start_vy + Apred_vx[2]*    sim.start_psiDot + Bpred_vx*acc       + Cpred_vx
            vy_next     = Apred_vy[0]    *vx + Apred_vy[1]*    sim.start_vy + Apred_vy[2]*    sim.start_psiDot + Bpred_vy*delta     + Cpred_vy
            psiDot_next = Apred_psiDot[0]*vx + Apred_psiDot[1]*sim.start_vy + Apred_psiDot[2]*sim.start_psiDot + Bpred_psiDot*delta + Cpred_psiDot

            vy_outPred.append(Apred_vy[0]*vx + Apred_vy[1]*sim.start_vy + Apred_vy[2]*sim.start_psiDot + Bpred_vy*delta + Cpred_vy)
            vy_outNext.append(Apred_vy[0]*vx_next + Apred_vy[1]*vy_next + Apred_vy[2]*psiDot_next + Bpred_vy*delta + Cpred_vy)

            psiDot_outTrue.append(sim.psiDot)
            psiDot_outPred.append(Apred_psiDot[0]*vx + Apred_psiDot[1]*sim.start_vy + Apred_psiDot[2]*sim.start_psiDot + Bpred_psiDot*delta + Cpred_psiDot)

        plt.figure()
        plt.plot(vx_in, psiDot_outTrue, '-o', color='r', label='psiDot_outTrue')
        plt.plot(vx_in, psiDot_outPred, '-s', color='b', label='psiDot_outPred')
        plt.ylabel('vy [m/s]')
        plt.xlabel('vx [rad]')
        plt.legend()

        plt.figure()
        plt.plot(vx_in, vy_outTrue, '-o', color='r', label='vy_outTrue')
        plt.plot(vx_in, vy_outPred, '-s', color='b', label='vy_outPred')
        plt.plot(vx_in, vy_outTrue_1, '-o', color='k', label='vy_outTrue_1')
        plt.plot(vx_in, vy_outNext, '-s', color='g', label='vy_outNext')
        plt.ylabel('vy [m/s]')
        plt.xlabel('vx [rad]')
        plt.legend()
        



        # Plot track
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

        plt.plot(sim.start_x, sim.start_y, 'sr')
        
        plt.show()

def simulateForCostDelta(sim, delta, acc):
    u = np.zeros(2)
    u[0] = acc
    u[1] = delta

    time = 0
    while (time < 10):
        sim.f(u)
        time = time + 1

    print time
    return sim.vy

def simulateForCostDelta_1(sim, delta, acc):
    u = np.zeros(2)
    u[0] = acc
    u[1] = delta

    time = 0
    while (time < 20):
        sim.f(u)
        time = time + 1

    print time
    return sim.vy



def plotSteeringGain(LMPController, map, LapToPlot):
    plotColors = ['b','g','r','c','y','k','k','k','k','k','k','k','k','k','k','k']

    SS_glob = LMPController.SS_glob
    LapCounter  = LMPController.LapCounter
    SS      = LMPController.SS
    uSS     = LMPController.uSS

    plt.figure()
    plt.subplot(211)

    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:(LapCounter[i]-1), 4, i], LMPController.steeringGain_vy[0:(LapCounter[i] - 1), 0, i], '--o', color=plotColors[counter], label=i)
        counter += 1
    plt.ylabel('Steering Gain [rad]')
    plt.xlabel('s [m]')
    plt.legend(bbox_to_anchor=(0,1.02,1,0.2), borderaxespad=0, ncol=len(LapToPlot))

    plt.subplot(212)
    counter = 0
    for i in LapToPlot:
        plt.plot(SS[0:(LapCounter[i]-1), 4, i], LMPController.steeringGain_vy[0:(LapCounter[i] - 1), 1, i], '--o', color=plotColors[counter], label=i)
        counter += 1
    plt.ylabel('Velocity Gain [rad]')
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


    plotColors = ['r','g','b','y','k','k','k','k','k','k','k','k']

    plt.figure()
    counter = 0
    for i in LapToPlot:
        indexToUse = np.argsort(FyF[0:LapCounter[i]-1, 0, i])
        plt.plot(slipF[indexToUse, 0, i], FyF[indexToUse, 0, i], '-', linewidth=7.0, color=plotColors[counter], label=i)
        counter += 1
    plt.legend()

    plt.ylabel('Lateral Force [N]')
    plt.xlabel('Slip Angle [deg]')

    plt.figure()
    counter = 0
    for i in LapToPlot:
        indexToUse = np.argsort( FyR[0:LapCounter[i]-1, 0, i])
        plt.plot(slipR[indexToUse, 0, i], FyR[indexToUse, 0, i], '-', color=plotColors[counter],linewidth=7.0, label=i)
        counter += 1

    plt.annotate('local max', xy=(2, 1), xytext=(3, 1.5), arrowprops=dict(facecolor='black', shrink=0.05))
    plt.legend()
    plt.ylabel('Lateral Force [N]')
    plt.xlabel('Slip Angle [deg]')


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


main()