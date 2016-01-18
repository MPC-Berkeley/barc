#!/home/mpc/.virtualenvs/research/bin/python

import rospy

import time
from data_connection import *
from base import *
import datetime
import pytz

from data_service.srv import *
from data_service.msg import *


if __name__ == '__main__':
    t = datetime.datetime.now(tz=pytz.UTC)
    tvec = [t + datetime.timedelta(seconds=i*.01) for i in range(10)]
    tsvec = [DataConnection.utc_to_millisec(at) for at in tvec]

    signal_vector =  [ats-tsvec[0] for ats in tsvec]

    time_signal_id = 'test_time'

    blob_signal_id = 'test_blob'

    rospy.wait_for_service('send_data')
    rospy.wait_for_service('register_experiment')

    # Register experiment
    try:
        register_experiment = rospy.ServiceProxy('register_experiment', RegisterExperiment)
        # reg_e = RegisterExperiment
        # reg_e.experiment = 'test1'
        response = register_experiment('test1')
        print response
    except rospy.ServiceException as e:
        print e

    # # Send time data
    # try:
    #     send_data = rospy.ServiceProxy('send_data', DataForward)
    #     time_signal = TimeSignal
    #     time_signal.id = time_signal_id
    #     time_signal.timestamps = tsvec
    #     time_signal.signal = signal_vector
    #     response = send_data(time_signal, None)
    # except rospy.ServiceException, e:
    #     print 'Call to service failed: %s' %e

    # Send custom data
    # try:
    #     send_data = rospy.ServiceProxy('send_data', DataForward)
    #     custom_signal = CustomSignal
    #     custom_signal.id = blob_signal_id
    #     custom_signal.signal = json.dumps({'sig1': [-1, -2, -3, -4], 'sig2': [1, 2, 3, 4]})
    #     # custom_signal.signal = [-1, -2, -3 -4]
    #     response = send_data(None, custom_signal)
    # except rospy.ServiceException, e:
    #     print 'Call to service failed: %s' %e

    # # Retrieve time data
    # try:
    #     retrieve_data = rospy.ServiceProxy('retrieve_data', DataRetrieve)
    #     response = retrieve_data(time_signal_id, True)
    #     print response
    # except rospy.ServiceException, e:
    #     print 'Call to service failed: %s' %e

    # Retrieve custom data
    # try:
    #     retrieve_data = rospy.ServiceProxy('retrieve_data', DataRetrieve)
    #     response = retrieve_data(blob_signal_id, False)
    #     print response
    # except rospy.ServiceException, e:
    #     print 'Call to service failed: %s' %e
