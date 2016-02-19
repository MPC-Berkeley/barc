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
    rospy.wait_for_service('register_setting')

    experiment_name = 'ex3'

    # generate and send signal
    try:
        t = datetime.datetime.now(tz=pytz.UTC)
        tvec = [t + datetime.timedelta(seconds=i*.01) for i in range(10)]
        tsvec = [DataConnection.utc_to_millisec(at) for at in tvec]
        # signal_vector =  [ats-tsvec[0] for ats in tsvec]
        signal_vector = [float(i)*15.3 for i in range(len(tsvec))]
        signal_vs = []

        for s in signal_vector:
            signal_vs.append([s, 3])

        send_data = rospy.ServiceProxy('send_data', DataForward)
        time_signal = TimeSignal()
        time_signal.name = 'mpc_sig2'
        time_signal.timestamps = signal_vector
        time_signal.signal = json.dumps(signal_vs)

        print "Time Signal", time_signal
        print "Experiment name", experiment_name

        response = send_data(time_signal, None, experiment_name)
    except rospy.ServiceException, e:
        print 'Call to service failed: %s' %e


    # # Register a setting
    register_setting = rospy.ServiceProxy('register_setting', RegisterSetting)

    register_setting('I', '0.20')
    register_setting('P', '0.60')
