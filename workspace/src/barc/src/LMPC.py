#!/usr/bin/env python

'''
    File name: LMPC.py
    Author: Ugo Rosolia
    Email: ugo.rosolia@berkeley.edu
    Python Version: 2.7.12
'''
import datetime
import rospy
from track_fnc import Map
from ControllerClasses import PID
from barc.msg import pos_info, ECU
import numpy as np
import pdb
import sys
import pickle
from Utilities import Regression
from PathFollowingLTI_MPC import PathFollowingLTI_MPC
from PathFollowingLTVMPC import PathFollowingLTV_MPC
from ClassControllerLMPC import ControllerLMPC

def main():
    # Initializa ROS node
    rospy.init_node("LMPC")
    input_commands = rospy.Publisher('ecu', ECU, queue_size=1)
    loop_rate = 10.0
    dt = 1.0/loop_rate
    rate = rospy.Rate(loop_rate)

    # Objects initializations
    cmd = ECU()                                              # Command message
    ClosedLoopData = ClosedLoopDataObj(0.1, 6000, 0)         # Closed-Loop Data
    estimatorData  = EstimatorData()
    map = Map()                                              # Map
    
    # Choose Controller and Number of Laps
    PickController = "LMPC"
    NumberOfLaps   = 10
    vt = 1.0
    PathFollowingLaps = 1
    ControllerPID = PID(vt) 
    Controller    = ControllerInitialization(PickController, NumberOfLaps, dt, vt, map)
         
    # Initialize variables for main loop
    GlobalState      = np.zeros(6)
    LocalState       = np.zeros(6)
    HalfTrack   = 0; LapNumber = 0; RunController = 1

    # Loop running at loop rate
    TimeCounter = 0
    KeyInput = raw_input("Press enter to start the controller... \n")
    while (not rospy.is_shutdown()) and RunController == 1:    
        # Read Measurements
        GlobalState[:] = estimatorData.CurrentState
        LocalState[:]  = estimatorData.CurrentState
        LocalState[4], LocalState[5], LocalState[3], insideTrack = map.getLocalPosition(GlobalState[4], GlobalState[5], GlobalState[3])

        # Check if the lap has finisched
        if LocalState[4] >= 3*map.TrackLength/4:
            HalfTrack = 1

        if HalfTrack == 1 and LocalState[4] <= map.TrackLength/4:
            HalfTrack = 0
            LapNumber += 1 
            print "Lap completed starting lap:", LapNumber, ". Lap time: ", float(TimeCounter)/loop_rate
            TimeCounter = 0
            if PickController == "LMPC":
                Controller.addTrajectory(ClosedLoopData)
                ClosedLoopData.updateInitialConditions(LocalState, GlobalState)

            if LapNumber >= NumberOfLaps:
                RunController = 0

        # If inside the track publish input
        if (insideTrack == 1):
            startTimer = datetime.datetime.now()
            if LapNumber < PathFollowingLaps :        # First path following lap
                ControllerPID.solve(LocalState)
                cmd.servo = ControllerPID.uPred[0,0]
                cmd.motor = ControllerPID.uPred[0,1]
            else:                                     # Else use the selected controller
                Controller.solve(LocalState)
                cmd.servo = Controller.uPred[0,0]
                cmd.motor = Controller.uPred[0,1]
                if (Controller.solverTime.total_seconds() + Controller.linearizationTime.total_seconds() > dt):
                    print "NOT REAL-TIME FEASIBLE!!!"
                    print "Solver time: ", Controller.solverTime.total_seconds(), " Linearization Time: ", Controller.linearizationTime.total_seconds()
            
            endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
            #print "Tot Solver Time: ", deltaTimer.total_seconds()

            # Publish input
            input_commands.publish(cmd)

            # Record data
            uApplied = np.array([cmd.servo, cmd.motor])
            ClosedLoopData.addMeasurement(GlobalState, LocalState, uApplied)

            # Add data to SS
            if (PickController == "LMPC") and (LapNumber >= PathFollowingLaps):
                Controller.addPoint(LocalState, uApplied, TimeCounter)

        else:                                        # If car out of the track
            cmd.servo = 0
            cmd.motor = 0
            input_commands.publish(cmd)

            print " Current Input: ", cmd.servo, cmd.motor
            print " X, Y State: ", GlobalState
            print " Current State: ", LocalState

        # Increase time counter and ROS sleep()
        TimeCounter = TimeCounter + 1
        rate.sleep()

    # Save Data
    file_data = open(sys.path[0]+'/data/ClosedLoopData'+PickController+'.obj', 'wb')
    pickle.dump(ClosedLoopData, file_data)
    file_data.close()

# ===============================================================================================================================
# ==================================================== END OF MAIN ==============================================================
# ===============================================================================================================================

def ControllerInitialization(PickController, NumberOfLaps, dt, vt, map):
    if PickController == 'PID':
        Controller = PID(vt)                                # PID controller
    elif PickController == "TI_MPC":
        file_data = open(sys.path[0]+'/data/ClosedLoopDataPID.obj', 'rb')
        ClosedLoopDataPID = pickle.load(file_data)
        file_data.close()
        lamb = 0.0000001
        A, B, Error = Regression(ClosedLoopDataPID.x, ClosedLoopDataPID.u, lamb)
        print "A matrix: \n", A
        print "B matrix: \n", B      
        Q = 1*np.diag([1.0, 1.0, 1, 10.0, 0.0, 10.0]) # vx, vy, wz, epsi, s, ey
        R = np.diag([1.0, 1.0]) # delta, a
        N = 12
        Controller = PathFollowingLTI_MPC(A, B, Q, R, N, vt)
    elif PickController == "TV_MPC":
        file_data = open(sys.path[0]+'/data/ClosedLoopDataPID.obj', 'rb')
        ClosedLoopDataPID = pickle.load(file_data)
        file_data.close()
        Q = 1*np.diag([1.0, 1.0, 1, 10.0, 0.0, 10.0]) # vx, vy, wz, epsi, s, ey
        R = np.diag([1.0, 1.0]) # delta, a
        N = 12
        Controller = PathFollowingLTV_MPC(Q, R, N, vt, ClosedLoopDataPID.x[0:ClosedLoopDataPID.SimTime, :], 
                                                       ClosedLoopDataPID.u[0:ClosedLoopDataPID.SimTime, :], dt, map, "OSQP")
    elif PickController == "LMPC":
        file_data = open(sys.path[0]+'/data/ClosedLoopDataPID.obj', 'rb')
        ClosedLoopDataPID = pickle.load(file_data)
        file_data.close()
        file_data = open(sys.path[0]+'/data/ClosedLoopDataTV_MPC.obj', 'rb')
        ClosedLoopDataTV_MPC = pickle.load(file_data)
        file_data.close()
        Laps       = NumberOfLaps+2   # Total LMPC laps
        # Safe Set Parameters
        TimeLMPC   = 400              # Simulation time
        N = 12
        LMPC_Solver = "OSQP"          # Can pick CVX for cvxopt or OSQP. For OSQP uncomment line 14 in LMPC.py
        SysID_Solver = "scipy"          # Can pick CVX, OSQP or scipy. For OSQP uncomment line 14 in LMPC.py  
        numSS_it = 2                  # Number of trajectories used at each iteration to build the safe set
        numSS_Points = 32 + N         # Number of points to select from each trajectory to build the safe set
        shift = 0                     # Given the closed point, x_t^j, to the x(t) select the SS points from x_{t+shift}^j
        # Tuning Parameters
        Qslack  = 50*np.diag([10, 10, 10, 10, 10, 10])          # Cost on the slack variable for the terminal constraint
        Q_LMPC  =  0 * np.diag([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # State cost x = [vx, vy, wz, epsi, s, ey]
        R_LMPC  =  5 * np.diag([1.0, 1.0])                      # Input cost u = [delta, a]
        dR_LMPC =  1 * np.array([1.0, 1.0])                     # Input rate cost u
        Controller = ControllerLMPC(numSS_Points, numSS_it, N, Qslack, Q_LMPC, R_LMPC, dR_LMPC, 6, 2, shift, 
                                        dt, map, Laps, TimeLMPC, LMPC_Solver, SysID_Solver)
        Controller.addTrajectory(ClosedLoopDataPID)
        Controller.addTrajectory(ClosedLoopDataTV_MPC)
        print "Here!"

    return Controller       

class EstimatorData(object):
    """Data from estimator"""
    def __init__(self):
        """Subscriber to estimator"""
        rospy.Subscriber("pos_info", pos_info, self.estimator_callback)
        self.CurrentState = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    def estimator_callback(self, msg):
        """
        Unpack the messages from the estimator
        """
        self.CurrentState = [msg.v_x, msg.v_y, msg.psiDot, msg.psi, msg.x, msg.y]
        
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
        self.dt = dt
        self.Points = int(Time / dt)  # Number of points in the simulation
        self.u = np.zeros((self.Points, 2))  # Initialize the input vector
        self.x = np.zeros((self.Points + 1, 6))  # Initialize state vector (In curvilinear abscissas)
        self.x_glob = np.zeros((self.Points + 1, 6))  # Initialize the state vector in absolute reference frame
        self.SimTime = 0
        self.x[0,0] = v0
        self.x_glob[0,0] = v0
        self.CurrentState = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def updateInitialConditions(self, x, x_glob):
        """Clears memory and resets initial condition
        x: initial condition is the curvilinear reference frame
        x_glob: initial condition in the inertial reference frame
        """
        self.x[0, :] = x
        self.x_glob[0, :] = x_glob

        self.x[1:, :] = np.zeros((self.x.shape[0]-1, 6))
        self.x_glob[1:, :] = np.zeros((self.x.shape[0]-1, 6))
        self.SimTime = 0

    def addMeasurement(self, xMeasuredGlob, xMeasuredLoc, uApplied):
        """Add point to the object ClosedLoopData
        xMeasuredGlob: measured state in the inerial reference frame
        xMeasuredLoc: measured state in the curvilinear reference frame
        uApplied: input applied to the system
        """
        self.x[self.SimTime, :]      = xMeasuredLoc
        self.x_glob[self.SimTime, :] = xMeasuredGlob
        self.u[self.SimTime, :]      = uApplied
        self.SimTime = self.SimTime + 1
        

if __name__ == "__main__":

    try:    
        main()
        
    except rospy.ROSInterruptException:
        pass
