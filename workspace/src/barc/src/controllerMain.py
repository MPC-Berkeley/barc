#!/usr/bin/env python

'''
    File name: LMPC.py
    Author: Ugo Rosolia
    Email: ugo.rosolia@berkeley.edu
    Python Version: 2.7.12
'''
import os
import sys
sys.path.append(sys.path[0]+'/ControllersObject')
sys.path.append(sys.path[0]+'/Utilities')
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

    print "Track Length: ", map.TrackLength 
    
    # Choose Controller and Number of Laps
    InputDelay = 1 # 0 for no delay, 1 for 1 step delay, 2 for 2 steps delay

    PickController = "LMPC"
    NumberOfLaps   = 30
    vt = 1.3
    PathFollowingLaps = 2
    
    if mode == "simulations":
        PIDnoise = np.array([1.0, 1.0]) # noise on [Steering, Acceleration] 
    else:
        PIDnoise = np.array([1.0, 10.0]) # noise on [Steering, Acceleration] 

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
    uApplied = np.array([0.0, 0.0])
    oldU = np.array([0.0, 0.0])
    oldX = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


    firtLap = True
    while (not rospy.is_shutdown()) and RunController == 1:    
        # == Start: Read Measurements
        GlobalState[:] = estimatorData.CurrentState
        if (GlobalState == oldX).all():
            print "================ Package Loss! The state as not been updated! ==============="
        oldX[:] = GlobalState
        LocalState[:]  = estimatorData.CurrentState
        LocalState[4], LocalState[5], LocalState[3], insideTrack = map.getLocalPosition(GlobalState[4], GlobalState[5], GlobalState[3])
        # == End: Read Measurements

        # == Start: Check if the lap has finisched
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
                # if Controller.lapSelected != []:
                #     print "Lap Ordered: ", Controller.Qfun[0, Controller.lapSelected]
    
                print "Lap completed starting lap:", LapNumber, ". Lap time: ", float(Controller.LapTime)/loop_rate
                Controller.resetTime()

            if LapNumber >= NumberOfLaps:
                RunController = 0

            HalfTrack = 7
            TimeCounter = 0
        # == End: Check if the lap has finisched

        # If inside the track publish input
        if (insideTrack == 1):
            startTimer = datetime.datetime.now()

            # == Start: Computing Input
            if LapNumber < PathFollowingLaps :        # First path following lap
                ControllerLap0.solve(LocalState)
                cmd.servo = ControllerLap0.uPred[0,0]
                cmd.motor = ControllerLap0.uPred[0,1]
                # cmd.servo = 0.0 #ControllerLap0.uPred[0,0]
                # cmd.motor = 0.3 #ControllerLap0.uPred[0,1]

                oldU = uApplied
                uApplied = np.array([cmd.servo, cmd.motor])
                # Publish input
                input_commands.publish(cmd)

            elif PickController == "PID":
                Controller.solve(LocalState)
                cmd.servo = Controller.uPred[0,0]
                cmd.motor = Controller.uPred[0,1]
                oldU = uApplied
                uApplied = np.array([cmd.servo, cmd.motor])
                # Publish input
                input_commands.publish(cmd)
                
            else:                                     # Else use the selected controller
                if LocalState[0] < 0.5 and cmd.motor < 0.5:
                    cmd.motor = 0.5

                oldU = uApplied
                uApplied = np.array([cmd.servo, cmd.motor])

                # Publish input
                input_commands.publish(cmd)

                oneStepPredictionError = LocalState - oneStepPrediction # Subtract the local measurement to the previously predicted one step

                if InputDelay == 1:
                    oneStepPrediction, oneStepPredictionTime = Controller.oneStepPrediction(LocalState, uApplied, 1)
                elif InputDelay == 2:
                    oneStepPrediction, oneStepPredictionTime = Controller.oneStepPrediction(LocalState, oldU, 1)
                    oneStepPrediction, oneStepPredictionTime = Controller.oneStepPrediction(oneStepPrediction, uApplied, 1)

                # print "LocalState: ", LocalState
                # print "oneStepPrediction: ", oneStepPrediction

                Controller.solve(oneStepPrediction)
                cmd.servo = Controller.uPred[0,0]
                cmd.motor = Controller.uPred[0,1]
                if (Controller.solverTime.total_seconds() + Controller.linearizationTime.total_seconds() + oneStepPredictionTime.total_seconds() > dt):
                    print "NOT REAL-TIME FEASIBLE!!!"
                    print "Solver time: ", Controller.solverTime.total_seconds(), " Linearization Time: ", Controller.linearizationTime.total_seconds() + oneStepPredictionTime.total_seconds()
            
            endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
            #print "Tot Solver Time: ", deltaTimer.total_seconds()
            
            # == End: Computing Input

            

            # == Start: Record data
            if (PickController != "LMPC") and (LapNumber >= 1):
                LocalState[4] = LocalState[4] + (LapNumber - 1)*map.TrackLength
                if InputDelay == 1:
                    ClosedLoopData.addMeasurement(GlobalState, LocalState, uApplied)
                elif InputDelay ==2:
                    ClosedLoopData.addMeasurement(GlobalState, LocalState, oldU)
            elif LapNumber >= 1:
                if InputDelay == 1:
                    ClosedLoopData.addMeasurement(GlobalState, LocalState, uApplied)
                elif InputDelay ==2:
                    ClosedLoopData.addMeasurement(GlobalState, LocalState, oldU)
            # Add data to SS and Record Prediction

            if (PickController == "LMPC"):
                if LapNumber >= PathFollowingLaps and Controller.xPred != []:
                    OL_predictions.s    = Controller.xPred[:, 4]
                    OL_predictions.ey   = Controller.xPred[:, 5]
                    OL_predictions.epsi = Controller.xPred[:, 3]
                    pred_treajecto.publish(OL_predictions)

            if (PickController == "LMPC") and (LapNumber >= PathFollowingLaps):

                SS_glob_sel.SSx       = Controller.SS_glob_PointSelectedTot[4, :]
                SS_glob_sel.SSy       = Controller.SS_glob_PointSelectedTot[5, :]
                sel_safe_set.publish(SS_glob_sel)

                OpenLoopData.oneStepPredictionError[:,TimeCounter, Controller.it]   = oneStepPredictionError               
                OpenLoopData.PredictedStates[:,:,TimeCounter, Controller.it]        = Controller.xPred
                OpenLoopData.PredictedInputs[:, :, TimeCounter, Controller.it]      = Controller.uPred
                OpenLoopData.SSused[:, :, TimeCounter, Controller.it]               = Controller.SS_PointSelectedTot
                OpenLoopData.Qfunused[:, TimeCounter, Controller.it]                = Controller.Qfun_SelectedTot

                if InputDelay == 1:
                    Controller.addPoint(LocalState, GlobalState, uApplied, TimeCounter)
                elif InputDelay ==2:
                    Controller.addPoint(LocalState, GlobalState, oldU, TimeCounter)

            # == End: Record data
        else:                                        # If car out of the track
            cmd.servo = 0
            cmd.motor = 0
            input_commands.publish(cmd)

            print " Current Input: ", cmd.servo, cmd.motor
            print " X, Y State: ", GlobalState
            print " Current State: ", LocalState

        # Increase time counter and ROS sleep()
        TimeCounter = TimeCounter + 1
        if PickController == "LMPC":
            Controller.setTime(TimeCounter)
        rate.sleep()

    # == Start: Save Data
    if saveData == True:
        file_data = open(homedir+'/barc_data'+'/ClosedLoopData'+PickController+'.obj', 'wb')
        pickle.dump(ClosedLoopData, file_data)
        pickle.dump(Controller, file_data)
        pickle.dump(OpenLoopData, file_data)    

        file_data.close()
    # == End: Save Data

# ===============================================================================================================================
# ==================================================== END OF MAIN ==============================================================
# ===============================================================================================================================
def ControllerInitialization(PickController, NumberOfLaps, dt, vt, map, mode, PIDnoise):
    OpenLoopData = 0.0

    # TI MPC tuning
    if mode == "simulations":
        Q = 1*np.diag([500.0, 1.0, 10.0, 1.0, 0.0, 10 * 2 * 5 * 50.0]) # vx, vy, wz, epsi, s, ey
        R = np.diag([1.0, 1.0]) # delta, a
        N = 12
        TI_Qlane   =  1 * np.array([100, 0]) # Quadratic and linear slack lane cost
    else:
        Q = 1*np.diag([500.0, 1.0, 10.0, 10 * 5.0, 0.0, 250.0]) # vx, vy, wz, epsi, s, ey
        R = np.diag([1.0, 1.0]) # delta, a
        N = 12
        TI_Qlane   =  1 * np.array([100, 0]) # Quadratic and linear slack lane cost


    if PickController == 'PID':
        ControllerLap0 = PID(vt, PIDnoise, mode) 
    else:
        file_data = open(homedir+'/barc_data/'+'/ClosedLoopDataPID.obj', 'rb')
        ClosedLoopDataPID = pickle.load(file_data)
        file_data.close()
        if mode == "simulations":
            lamb = 0.0000001
        else:
            lamb = 0.0000001    
        A, B, Error = Regression(ClosedLoopDataPID.x, ClosedLoopDataPID.u, lamb)
        print "A matrix: \n", A
        print "B matrix: \n", B      
        ControllerLap0 = PathFollowingLTI_MPC(A, B, Q, R, N, vt, TI_Qlane)
        # ControllerLap0 = PID(vt, PIDnoise)

    if PickController == 'PID':
        Controller = PID(vt, PIDnoise, mode)
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
        if mode == "simulations":
            Qslack  = 5 * np.diag([10, 0.1, 1, 0.1, 10, 1])          # Cost on the slack variable for the terminal constraint
            Qlane   = 0.1 * 0.1 * 0.5 * 10 * np.array([50, 10]) # Quadratic slack lane cost
            Q_LMPC  =  0 * np.diag([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])  # State cost x = [vx, vy, wz, epsi, s, ey]
            R_LMPC  =  0 * np.diag([1.0, 1.0])                      # Input cost u = [delta, a]
            dR_LMPC =  1 * np.array([ 5 * 0.5 * 10.0, 8 * 20.0])                     # Input rate cost u
        else:
            Qslack  = 0.1 * 5 * np.diag([10, 0.1, 1, 0.1, 10, 1])          # Cost on the slack variable for the terminal constraint
            Qlane   = 0.1 * 0.5 * 10 * np.array([50, 10]) # Quadratic slack lane cost
            Q_LMPC  =  0 * np.diag([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])  # State cost x = [vx, vy, wz, epsi, s, ey]
            R_LMPC  =  0 * np.diag([1.0, 1.0])                      # Input cost u = [delta, a]
            dR_LMPC =  1 * np.array([ 2 * 5 * 0.5 * 10.0, 2 * 8 * 20.0])                     # Input rate cost u
        
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
    def __init__(self, vt, noise, mode):
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
        self.mode = mode

    def solve(self, x0):
        """Computes control action
        Arguments:
            x0: current state position
        """
        vt = self.vt
        if self.mode == "simulations":
            Steering = - 0.5 * 2.0 * x0[5] - 2 * 0.5 * x0[3] - 0.001 * self.integral[0]
            Accelera = 0.5 * 1.5 * (vt - x0[0]) + 0.1 * self.integral[1]
        else:
            Steering = - 0.5 * x0[5] - x0[3] - 0.1 * self.integral[0]
            Accelera = 1.5 * (vt - x0[0]) + 0.1 * self.integral[1]

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
