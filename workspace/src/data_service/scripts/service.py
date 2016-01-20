#!/usr/bin/env python

import rospy
import json
import datetime
import pytz

from std_msgs.msg import Float64MultiArray

from data_connection import *
from base import *

from data_service.srv import *
from data_service.msg import *


configurator = init_configurator()
data_connection = DataConnection(configurator)

signal_id_map = dict()
blob_id_map = dict()

response_ok = 'Ok'


def get_time_signal(signal_id):

    if signal_id not in signal_id_map:
        print 'Requesting new time signal'
        signal = data_connection.get_or_create_signal(signal_id)
        signal_id_map[signal_id] = signal

    return signal_id_map[signal_id]

def get_blob_signal(signal_id):

    if signal_id not in blob_id_map:
        print 'Requesting new blob signal'
        signal = data_connection.get_or_create_blob(signal_id)
        blob_id_map[signal_id] = signal

    return blob_id_map[signal_id]


def send_time_signal(time_signal, experiment_id):
    signal_id = time_signal.id
    #signal_id = 'sig_mpc3'
    signal = get_time_signal(signal_id)
    response = response_ok

    try:
        timestamps = time_signal.timestamps
        signals = json.loads(time_signal.signal)
        signal_points = []

        for ts, sig in zip(timestamps, signals):
#        for ts in timestamps:
            sig.append(ts)
            signal_points.append(sig)

        print signal_points

        data_connection.add_signal_points(signal['id'],
                                          signal_points)
    except Exception as e:
        print e
        response = str(e)

    return response


def send_custom_signal(custom_signal, experiment_id):
    signal = get_blob_signal(custom_signal.id)
    response = response_ok

    try:
        data_connection.set_blob_data(signal['id'], custom_signal.signal, experiment_id)
    except Exception as e:
        response = str(e)

    return response


def handle_send_data(req):
    response = response_ok

    if req.time_signal != None and req.time_signal.id != '':
        print 'SENDING DATAAA'
        response = send_time_signal(req.time_signal, req.experiment_id)
        if response != response_ok:
            return DataForwardResponse(response)

#    if req.custom_signal != None and req.custom_signal.id != '':
#        response = send_custom_signal(req.custom_signal)
#        if response != response_ok:
#            return DataForwardResponse(response)

    return DataForwardResponse(response)


def handle_retrieve_data(req):
    if req.id is None or req.id == '':
        return None

    signal_id = req.id

    signal_response = CustomSignal()
    signal_response.id = req.id

    if req.is_time:
        signal = get_time_signal(signal_id)
        try:
            response = data_connection.get_signal_points(signal['id'])
            signal_response.signal = json.dumps(response)
        except Exception as e:
            signal_response.signal = str(e)
            return DataRetrieveResponse(signal_response)
    else:
        signal = get_blob_signal(signal_id)
        try:
            response = data_connection.get_blob_data(signal['id'])
            signal_response.signal = response
        except Exception as e:
            signal_response.signal = str(e)
            return DataRetrieveResponse(signal_response)

    return DataRetrieveResponse(signal_response)


def handle_register_experiment(req):
    response = data_connection.register_experiment(req.experiment)
    return RegisterExperimentResponse(response)


def send_data_service():
    rospy.init_node('data_service', anonymous=True)
    s1 = rospy.Service('send_data', DataForward, handle_send_data)
    s2 = rospy.Service('retrieve_data', DataRetrieve, handle_retrieve_data)
    e1 = rospy.Service('register_experiment', RegisterExperiment, handle_register_experiment)
    rospy.spin()


if __name__ == "__main__":
    send_data_service()
