#!/usr/bin/env python

from numpy import array, zeros, ones, sin, cos, vstack, matrix, roll
import numpy as np

class SimData:
    def __init__(self):
        # current state
        self.x          = 0
        self.y          = 0
        self.psi        = 0

        # state history
        self.N          = 20
        self.x_hist     = []
        self.y_hist     = []
        self.psi_hist   = []

        # mpc solution
        self.z1OL       = 0
        self.z2OL       = 0
        self.z3OL       = None
        self.z4OL       = None
        self.u1OL       = None
        self.u2OL       = None

        # vehicle corners
        self.Lf         = None
        self.c          = None
        self.xVhCorners0    = None      # (x,y) coordinates of vehicle corners at origin
        self.yVhCorners0    = None
        self.xyVhCorners0   = None
        self.xc         = None          # current (x,y) coordinates of vehicle corners
        self.yc         = None

    def setInitialState(self, s0):
        self.x      = s0[0]
        self.y      = s0[1]
        self.psi    = s0[2]

        self.x_hist     = self.x * ones(self.N)
        self.y_hist     = self.y * ones(self.N)
        self.psi_hist   = self.psi * ones(self.N)

    def setVehicleParameters(self,L,c):
        Lf              = L / 2.0
        self.Lf         = Lf
        self.c          = c

        # (x,y) coordinates of corners of vehicle
        self.xVhCorners      = array( [Lf, Lf, -Lf, -Lf, Lf] )
        self.yVhCorners      = array( [c, -c, -c, c, c] )
        self.xyVhCorners = vstack((self.xVhCorners, self.yVhCorners))

    def updateVhCorners(self):
        psi     = self.psi
        x       = self.x
        y       = self.y

        R                   = matrix([[cos(psi), -sin(psi)], [sin(psi), cos(psi)]])
        rotatedCorners      = R * self.xyVhCorners
        xCorners            = array(rotatedCorners[0,:])[0] + x
        yCorners            = array(rotatedCorners[1,:])[0] + y

        self.xc = xCorners
        self.yc = yCorners

    def stateCallback(self, msg):
        # update state
        self.x      = msg.x
        self.y      = msg.y
        self.psi    = msg.psi

        # update history
        self.x_hist         = roll(self.x_hist,-1)
        self.y_hist         = roll(self.y_hist,-1)
        self.psi_hist       = roll(self.psi_hist,-1)
        self.x_hist[-1]     = msg.x
        self.y_hist[-1]     = msg.y
        self.psi_hist[-1]   = msg.psi

    def mpcCallback(self,msg):
        self.z1OL    = msg.z1OL
        self.z2OL    = msg.z2OL
        self.z3OL    = msg.z3OL
        self.z4OL    = msg.z4OL
        self.u1OL    = msg.u1OL
        self.u2OL    = msg.u2OL