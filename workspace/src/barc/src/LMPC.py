#!/usr/bin/env python

'''
    File name: LMPC.py
    Author: Ugo Rosolia
    Email: ugo.rosolia@berkeley.edu
    Python Version: 2.7.12
'''

import rospy
from track_fnc import Map
from ControllerClasses import PID
from barc.msg import pos_info, ECU
import numpy as np
import pdb


def main():
    rospy.init_node("LMPC")

    input_commands = rospy.Publisher('ecu', ECU, queue_size=1)
    cmd = ECU()

    loop_rate = 10
    rate = rospy.Rate(loop_rate)

    ClosedLoopData = ClosedLoopDataObj(0.1, 10000, 0)

    pid = PID(1.0)

    map = Map()

    print "Controller is running"

    KeyInput = raw_input("Press enter to start controller...")

    GlobalState = np.zeros(6)
    LocalState  = np.zeros(6)

    while not rospy.is_shutdown():    
        GlobalState[:] = ClosedLoopData.CurrentState
        LocalState[:]  = ClosedLoopData.CurrentState
        LocalState[4], LocalState[5], LocalState[3], insideTrack = map.getLocalPosition(GlobalState[4], GlobalState[5], GlobalState[3])

        if insideTrack == 1:
            pid.solve(LocalState)
            cmd.servo = pid.uPred[0,0]
            cmd.motor = pid.uPred[0,1]
            input_commands.publish(cmd)
            # print " Current State: ", LocalState
        else:
            cmd.servo = 0
            cmd.motor = 0
            input_commands.publish(cmd)

            print " Current Input: ", cmd.servo, cmd.motor
            print " X, Y State: ", GlobalState
            print " Current State: ", LocalState


class ClosedLoopDataObj():
    """Object collecting closed loop data points
    Attributes:
        updateInitialConditions: function which updates initial conditions and clear the memory
    """
    def __init__(self, dt, Time, v0):
        """Initialization
        Arguments:
            dt: discretization time
            Time: maximum time [s] which can be recorded
            v0: velocity initial condition
        """
        rospy.Subscriber("pos_info", pos_info, self.estimator_callback)

        self.dt = dt
        self.Points = int(Time / dt)  # Number of points in the simulation
        self.u = np.zeros((self.Points, 2))  # Initialize the input vector
        self.x = np.zeros((self.Points + 1, 6))  # Initialize state vector (In curvilinear abscissas)
        self.x_glob = np.zeros((self.Points + 1, 6))  # Initialize the state vector in absolute reference frame
        self.SimTime = 0.0
        self.x[0,0] = v0
        self.x_glob[0,0] = v0
        self.CurrentState = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Counter = 0

    def updateInitialConditions(self, x, x_glob):
        """Clears memory and resets initial condition
        x: initial condition is the curvilinear reference frame
        x_glob: initial condition in the inertial reference frame
        """
        self.x[0, :] = x
        self.x_glob[0, :] = x_glob

        self.x[1:, :] = 0*self.x[1:, :]
        self.x_glob[1:, :] = 0*self.x_glob[1:, :]
        self.CurrentState = np.zeros(6)


    def estimator_callback(self, msg):
        """
        Unpack the messages from the estimator
        """
        self.x_glob[self.Counter, :] = [msg.v_x, msg.v_y, msg.psiDot, msg.psi, msg.x, msg.y]
        self.CurrentState = [msg.v_x, msg.v_y, msg.psiDot, msg.psi, msg.x, msg.y]
        self.Counter = self.Counter + 1

    def getCurrentState(self):
        return self.CurrentState


if __name__ == "__main__":

    try:
        main()
        
    except rospy.ROSInterruptException:
        pass
