#!/usr/bin/python
import time
import serial
from arduino_Interface import Controller
from maneuvers import CircularTest, DoubleLaneChange, Straight, SineSweep
import os
from math import pi

#############################################################
def main_auto():
	# launch arduino communication 
    tire_radius 		= 0.036*2*pi
    rateHz 				= 10
    controller_model 	= Controller(arduino_port = '/dev/ttyUSB0', 
			  						 baudRate = 1152000, 
									 sampleRate = rateHz, 
									 radius = tire_radius)
	# serial communication hack
    os.system("/usr/bin/arduino")

    # specify tests
    test_opt    = { 0 : CircularTest,
                    1 : DoubleLaneChange,
                    2 : Straight,
		    		3 : SineSweep   }

    test_sel    = 0
    test_mode   = test_opt.get(test_sel)

    test_mode(controller_model, 0, 1)

    time.sleep(1)
    test_mode(controller_model, 4, 1)

    time.sleep(3)
    test_mode(controller_model, 9, 1)

#############################################################
if __name__ == '__main__':
    print('starting run .... ')
    main_auto()
