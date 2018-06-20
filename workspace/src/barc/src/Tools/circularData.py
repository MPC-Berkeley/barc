import sys
sys.path.append(sys.path[0]+'/../Utilities')
from dataStructures import ClosedLoopDataObj

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import sys
import pickle
import pdb
from trackInitialization import Map
from dataStructures import LMPCprediction, EstimatorData, ClosedLoopDataObj
import os
import scipy.io as sio

def main():
    homedir = os.path.expanduser("~")

    file_data = open(homedir+'/barc_data/ClosedLoopDataCircularTest.obj', 'rb')    
    ClosedLoopData = pickle.load(file_data)
    file_data.close()
    map = Map("circle")

    plotTrajectory(map, ClosedLoopData)

    plt.show()


def plotTrajectory(map, ClosedLoop):
    x = ClosedLoop.x
    x_glob = ClosedLoop.x_glob
    u = ClosedLoop.u

    sio.savemat("/home/ugo/Desktop/circular_test_data.mat",{'x':x, 'x_glob': x_glob})
    
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

    index1 = (x[:, 0] >= 0.5) & (x[:, 0] <= 1.25)
    index2 = (x[:, 0] >= 1.25) & (x[:, 0] <= 2.1)
    index3 = (x[:, 0] >= 2.1) & (x[:, 0] <= 2.8)
    
    plt.plot(x_glob[index1, 4], x_glob[index1, 5], '-ro')
    plt.plot(x_glob[index2, 4], x_glob[index2, 5], '-ko')
    plt.plot(x_glob[index3, 4], x_glob[index3, 5], '-go')
    plt.xlabel('x')
    plt.ylabel('y')

    plt.figure()
    plt.plot(x[index1, 4], x[index1, 0], '-ro')
    plt.plot(x[index2, 4], x[index2, 0], '-ko')
    plt.plot(x[index3, 4], x[index3, 0], '-go')
    plt.xlabel('s')
    plt.ylabel('v')

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


main()