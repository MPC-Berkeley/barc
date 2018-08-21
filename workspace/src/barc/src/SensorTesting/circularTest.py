#!/usr/bin/env python

'''
    File name: LMPC.py
    Author: Ugo Rosolia
    Email: ugo.rosolia@berkeley.edu
    Python Version: 2.7.12
'''
import os
import sys
sys.path.append(sys.path[0]+'/RacingLMPC/ControllersObject')
sys.path.append(sys.path[0]+'/RacingLMPC/Utilities')
import datetime
import rospy
from trackInitialization import Map
from barc.msg import pos_info, ECU, prediction, SafeSetGlob
import numpy as np
import pdb
import sys
import pickle
from utilities import Regression
from dataStructures import LMPCprediction, EstimatorData, ClosedLoopDataObj
from PathFollowingLTI_MPC import PathFollowingLTI_MPC
from PathFollowingLTVMPC import PathFollowingLTV_MPC
from dataStructures import LMPCprediction, EstimatorData, ClosedLoopDataObj
from LMPC import ControllerLMPC
homedir = os.path.expanduser("~")    

def main():
    # Initializa ROS node
    rospy.init_node("LMPC")

    input_commands = rospy.Publisher('ecu', ECU, queue_size=1)
    pred_treajecto = rospy.Publisher('OL_predictions', prediction, queue_size=1)
    pred_treajecto = rospy.Publisher('OL_predictions', prediction, queue_size=1)
    sel_safe_set   = rospy.Publisher('SS', SafeSetGlob, queue_size=1)

    mode = rospy.get_param("/control/mode")
    saveData = rospy.get_param("/control/saveData")


    loop_rate = 10.0
    dt = 1.0/loop_rate
    rate = rospy.Rate(loop_rate)

    # Objects initializations
    SS_glob_sel    = SafeSetGlob()
    OL_predictions = prediction()
    cmd = ECU()                                              # Command message
    cmd.servo = 0.0
    cmd.motor = 0.0
    ClosedLoopData = ClosedLoopDataObj(0.1, 6000, 0)         # Closed-Loop Data
    estimatorData  = EstimatorData()
    map = Map()                                              # Map
    
    # Choose Controller and Number of Laps
    InputDelay = 1 # 0 for no delay, 1 for 1 step delay, 2 for 2 steps delay

    NumberOfLaps   = 30
    vt = 1.2
    PathFollowingLaps = 2
    
    if mode == "simulations":
        PIDnoise = np.array([1.0, 1.0]) # noise on [Steering, Acceleration] 
    else:
        PIDnoise = np.array([0.2, 1.0]) # noise on [Steering, Acceleration] 
    
    PickController = "PID"

    ControllerLap0, Controller,  OpenLoopData   = ControllerInitialization(PickController, NumberOfLaps, dt, vt, map, mode, PIDnoise)
                 
    # Initialize variables for main loop
    GlobalState      = np.zeros(6)
    LocalState       = np.zeros(6)
    HalfTrack   = 0; LapNumber = 0; RunController = 1
    
    if saveData == False:
        print "======================== Not saving data for this esperiment ========================"

    # Loop running at loop rate
    TimeCounter = 0
    KeyInput = raw_input("Press enter to start the controller... \n")
    oneStepPrediction = np.zeros(6)
    oldU = np.array([0.0, 0.0])
    oldX = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


    firtLap = True
    while (not rospy.is_shutdown()) and RunController == 1:    
        # Read Measurements
        GlobalState[:] = estimatorData.CurrentState
        if (GlobalState == oldX).all():
            print "================ Package Loss! The state as not been updated! ==============="
        oldX[:] = GlobalState
        LocalState[:]  = estimatorData.CurrentState
        LocalState[4], LocalState[5], LocalState[3], insideTrack = map.getLocalPosition(GlobalState[4], GlobalState[5], GlobalState[3])

        # Check if the lap has finisched
        if LocalState[4] >= 3*map.TrackLength/4:
            HalfTrack = 1

        if LocalState[4] >= 1*map.TrackLength/4 and LocalState[4] <= 3*map.TrackLength/4:
            firtLap = False

        if HalfTrack == 1 and LocalState[4] <= map.TrackLength/4 and firtLap == False:

            # If not LMPC then reset initial conditon after Lap 0
            if LapNumber == 0 and PickController != "LMPC":
                ClosedLoopData.updateInitialConditions(LocalState, GlobalState)

            # If LMPC put in SS lap after finisching lap 1
            if LapNumber >= 1 and PickController == "LMPC":
                Controller.addTrajectory(ClosedLoopData)
                ClosedLoopData.updateInitialConditions(LocalState, GlobalState)

            LapNumber += 1

            print "Lap completed starting lap:", LapNumber, ". Lap time: ", float(TimeCounter)/loop_rate
            if PickController == "LMPC":
                print "Lap completed starting lap:", LapNumber, ". Lap time: ", float(Controller.LapTime)/loop_rate
                Controller.resetTime()

            if LapNumber >= NumberOfLaps:
                RunController = 0

            HalfTrack = 0
            TimeCounter = 0

        # If inside the track publish input
        if (insideTrack == 1):
            startTimer = datetime.datetime.now()
            if LapNumber < 2 :        # First path following lap
                ControllerLap0.vt = 1
                ControllerLap0.solve(LocalState)
                cmd.servo = 1.5*0.15 #ControllerLap0.uPred[0,0]
                cmd.motor = ControllerLap0.uPred[0,1]

                uApplied = np.array([cmd.servo, cmd.motor])
                # Publish input
                input_commands.publish(cmd)
            elif LapNumber < 4:
                Controller.vt = 2.0
                Controller.solve(LocalState)
                cmd.servo = 1.5*0.15 #ControllerLap0.uPred[0,0]
                cmd.motor = Controller.uPred[0,1]
                uApplied = np.array([cmd.servo, cmd.motor])
                # Publish input
                input_commands.publish(cmd)
            elif LapNumber < 8:
                Controller.vt = 3.0
                Controller.solve(LocalState)
                cmd.servo = 1.5*0.15 #ControllerLap0.uPred[0,0]
                cmd.motor = Controller.uPred[0,1]
                uApplied = np.array([cmd.servo, cmd.motor])
                # Publish input
                input_commands.publish(cmd)
            elif LapNumber == 8:
                break
                
            endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
            #print "Tot Solver Time: ", deltaTimer.total_seconds()

            

            # Record data
            ClosedLoopData.addMeasurement(GlobalState, LocalState, uApplied)
            
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
    if saveData == True:
        file_data = open(homedir+'/barc_data/ClosedLoopDataCircularTest.obj', 'wb')
        pickle.dump(ClosedLoopData, file_data)
        pickle.dump(Controller, file_data)
        pickle.dump(OpenLoopData, file_data)    

        file_data.close()

# ===============================================================================================================================
# ==================================================== END OF MAIN ==============================================================
# ===============================================================================================================================
def ControllerInitialization(PickController, NumberOfLaps, dt, vt, map, mode, PIDnoise):
    OpenLoopData = 0.0

    # TI MPC tuning
    Q = 1*np.diag([500.0, 1.0, 10.0, 1.0, 0.0, 2 * 5 * 50.0]) # vx, vy, wz, epsi, s, ey
    R = np.diag([1.0, 1.0]) # delta, a
    N = 12
    TI_Qlane   =  1 * np.array([100, 0]) # Quadratic and linear slack lane cost


    if PickController == 'PID':
        ControllerLap0 = PID(vt, PIDnoise) 
    else:
        file_data = open(homedir+'/barc_data/'+'/ClosedLoopDataPID.obj', 'rb')
        ClosedLoopDataPID = pickle.load(file_data)
        file_data.close()
        lamb = 0.0000001
        A, B, Error = Regression(ClosedLoopDataPID.x, ClosedLoopDataPID.u, lamb)
        print "A matrix: \n", A
        print "B matrix: \n", B      
        ControllerLap0 = PathFollowingLTI_MPC(A, B, Q, R, N, vt, TI_Qlane)
        # ControllerLap0 = PID(vt, PIDnoise)

    if PickController == 'PID':
        Controller = PID(1.5*vt, PIDnoise)
                                        # PID controller
    elif PickController == "TI_MPC":
        file_data = open(homedir+'/barc_data/'+'/ClosedLoopDataPID.obj', 'rb')
        ClosedLoopDataPID = pickle.load(file_data)
        file_data.close()     
        Controller = PathFollowingLTI_MPC(A, B, Q, R, N, vt, TI_Qlane)

    elif PickController == "TV_MPC":
        file_data = open(homedir+'/barc_data/'+'/ClosedLoopDataPID.obj', 'rb')
        ClosedLoopDataPID = pickle.load(file_data)
        file_data.close()
        Q = 1*np.diag([10.0, 1.0, 1, 10.0, 0.0, 10.0]) # vx, vy, wz, epsi, s, ey
        R = np.diag([1.0, 5.0]) # delta, a
        N = 12
        Controller = PathFollowingLTV_MPC(Q, R, N, vt, ClosedLoopDataPID.x[0:ClosedLoopDataPID.SimTime, :], 
                                                       ClosedLoopDataPID.u[0:ClosedLoopDataPID.SimTime, :], dt, map, "OSQP")
    elif PickController == "LMPC":
        file_data = open(homedir+'/barc_data/'+'/ClosedLoopDataTI_MPC.obj', 'rb')
        ClosedLoopDataTI_MPC = pickle.load(file_data)
        file_data.close()
        Laps       = NumberOfLaps+2   # Total LMPC laps
        # Safe Set Parameters
        flag_LTV = True
        TimeLMPC   = 400              # Simulation time
        N = 12
        LMPC_Solver = "OSQP"          # Can pick CVX for cvxopt or OSQP. For OSQP uncomment line 14 in LMPC.py
        SysID_Solver = "scipy"        # Can pick CVX, OSQP or scipy. For OSQP uncomment line 14 in LMPC.py  
        numSS_it = 2                  # Number of trajectories used at each iteration to build the safe set
        numSS_Points = 42 + N         # Number of points to select from each trajectory to build the safe set
        shift = N / 2                     # Given the closed point, x_t^j, to the x(t) select the SS points from x_{t+shift}^j
        # Tuning Parameters
        Qslack  =  5 * np.diag([10, 1, 1, 1, 10, 1])          # Cost on the slack variable for the terminal constraint
        Qlane   = 0.5 * 10 * np.array([50, 0]) # Quadratic and linear slack lane cost
        Q_LMPC  =  0 * np.diag([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])  # State cost x = [vx, vy, wz, epsi, s, ey]
        R_LMPC  =  0 * np.diag([1.0, 1.0])                      # Input cost u = [delta, a]
        dR_LMPC =  1 * np.array([10 * 0.5 * 10.0, 8 * 20.0])                     # Input rate cost u
        Controller = ControllerLMPC(numSS_Points, numSS_it, N, Qslack, Qlane, Q_LMPC, R_LMPC, dR_LMPC, 6, 2, shift, 
                                        dt, map, Laps, TimeLMPC, LMPC_Solver, SysID_Solver, flag_LTV)
        # Controller.addTrajectory(ClosedLoopDataPID)
        Controller.addTrajectory(ClosedLoopDataTI_MPC)
        OpenLoopData = LMPCprediction(N, 6, 2, TimeLMPC, numSS_Points, Laps)
        print "LMPC initialized!"

    return ControllerLap0, Controller, OpenLoopData       
        
class PID:
    """Create the PID controller used for path following at constant speed
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, vt, noise):
        """Initialization
        Arguments:
            vt: target velocity
        """
        self.vt = vt
        self.uPred = np.zeros([1,2])

        startTimer = datetime.datetime.now()
        endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
        self.solverTime = deltaTimer
        self.linearizationTime = deltaTimer
        self.feasible = 1
        self.integral = np.array([0.0, 0.0])
        self.noise = noise

    def solve(self, x0):
        """Computes control action
        Arguments:
            x0: current state position
        """
        vt = self.vt
        Steering = - 0.5 * 2.0 * x0[5] - 2 * 0.5 * x0[3] - 0.001 * self.integral[0]
        Accelera = 0.5 * 1.5 * (vt - x0[0]) + 0.1 * self.integral[1]

        self.integral[0] = self.integral[0] +  0.1 * x0[5] + 0.1 * x0[3]
        self.integral[1] = self.integral[1] +  (vt - x0[0])

        self.uPred[0, 0] = self.truncate(Steering, 0.3) + self.truncate( np.random.randn() * 0.25 * self.noise[0], 0.3)
        self.uPred[0, 1] = self.truncate(Accelera, 2.0) + self.truncate( np.random.randn() * 0.1 * self.noise[0], 0.3)


    def truncate(self, val, bound):
        return np.maximum(-bound, np.minimum(val, bound))


if __name__ == "__main__":

    try:    
        main()
        
    except rospy.ROSInterruptException:
        pass
