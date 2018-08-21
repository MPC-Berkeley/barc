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
from ZeroStepLMPC import ControllerZeroStepLMPC
import time

homedir = os.path.expanduser("~")    

def main():
    # Initializa ROS node
    rospy.init_node("lowLevelPub")
    loop_rate = 100.0
    dt = 1.0/loop_rate
    rate = rospy.Rate(loop_rate)

    input_commands = rospy.Publisher('ecu', ECU, queue_size=1)
    cmd = ECU()

    LMPC = LMPC_commands()

    cntrStarted = False
    while (not rospy.is_shutdown()):
        if LMPC.motor > 0.0:
            cntrStarted = True

        if cntrStarted == True:
            cmd.servo = LMPC.servo
            cmd.motor = LMPC.motor
            input_commands.publish(cmd)

        rate.sleep()

class LMPC_commands(object):
    def __init__(self):
        rospy.Subscriber('ecu_LMPC', ECU, self.ecu_LMPC_callback, queue_size=1)

        # ENC measurement
        self.servo = 0.0
        self.motor = 0.0

    def ecu_LMPC_callback(self,data):
        """Unpack message from sensor, ENC"""
        self.servo = data.servo
        self.motor = data.motor

if __name__ == "__main__":

    try:    
        main()
        
    except rospy.ROSInterruptException:
        pass
