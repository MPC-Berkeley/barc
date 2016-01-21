#!/usr/bin/env python
import rospy

import time
from data_connection import *
from base import *
import datetime
import pytz

from data_service.srv import *
from data_service.msg import *

from std_msgs.msg import Float64MultiArray


import json

if __name__ == '__main__':
	
    rospy.wait_for_service('send_data')
    rospy.wait_for_service('register_experiment')

    experiment_id = 'test1'

    # Register experiment
    #try:
    #    register_experiment = rospy.ServiceProxy('register_experiment', RegisterExperiment)
    #    experiment_id = register_experiment('test1')
    #except rospy.ServiceException as e:
    #    print e

    # generate and send signal

    
    try:
		# generate data
        t = datetime.datetime.now(tz=pytz.UTC)
        tvec = [t + datetime.timedelta(seconds=i*.01) for i in range(10)]
        tsvec = [DataConnection.utc_to_millisec(at) for at in tvec]
        signal_vector =  [ats-tsvec[0] for ats in tsvec]
        signal_vs = []

        for s in signal_vector:
            signal_vs.append([s, 3])
		
        send_data = rospy.ServiceProxy('send_data', DataForward)
        time_signal = TimeSignal()
        time_signal.id = 'mpc_sig1'
        time_signal.timestamps = tsvec
        time_signal.signal = json.dumps(signal_vs)
		
        print "Time Signal", time_signal
        print "Experiment ID", experiment_id
		
		# Send signal
        response = send_data(time_signal, None, '')
    except rospy.ServiceException, e:
        print 'Call to service failed: %s' %e
    
    

    """
    # Send custom data
    try:
        send_data = rospy.ServiceProxy('send_data', DataForward)
        custom_signal = CustomSignal
        custom_signal.id = blob_signal_id
        custom_signal.signal = json.dumps({'sig1': [-1, -2, -3, -4], 'sig2': [1, 2, 3, 4]})
        # custom_signal.signal = [-1, -2, -3 -4]
        response = send_data(None, custom_signal)
    except rospy.ServiceException, e:
        print 'Call to service failed: %s' %e
	"""

    """
    time_signal_id = 'sig10'
 
    # Retrieve time data
    try:
        retrieve_data = rospy.ServiceProxy('retrieve_data', DataRetrieve)
        response = retrieve_data(time_signal_id, True)
        print response
    except rospy.ServiceException, e:
        print 'Call to service failed: %s' %e
    """

    # Retrieve custom data
    # try:
    #     retrieve_data = rospy.ServiceProxy('retrieve_data', DataRetrieve)
    #     response = retrieve_data(blob_signal_id, False)
    #     print response
    # except rospy.ServiceException, e:
    #     print 'Call to service failed: %s' %e
