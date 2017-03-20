#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------#!/usr/bin/env python

import time

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

signal_name_map = dict()
blob_name_map = dict()
experiment_name_map = dict()

response_ok = 'Ok'


def get_experiment(experiment_name):

    if experiment_name not in experiment_name_map:
        print 'Requesting new experiment'
        experiment = data_connection.get_or_create_experiment(experiment_name)
        experiment_name_map[experiment_name] = experiment

    return experiment_name_map[experiment_name]


def get_time_signal(signal_name, experiment_name):
    map_key = (signal_name, experiment_name)

    experiment = get_experiment(experiment_name)

    if map_key not in signal_name_map:
        print 'Requesting new time signal'
        signal = data_connection.get_or_create_signal(signal_name, experiment)
        signal_name_map[map_key] = signal

    return signal_name_map[map_key]


def get_blob_signal(signal_name):

    if signal_name not in blob_name_map:
        print 'Requesting new blob signal'
        signal = data_connection.get_or_create_blob(signal_name)
        blob_name_map[signal_name] = signal

    return blob_name_map[signal_name]


def send_time_signal(time_signal, experiment_name):
    signal_name = time_signal.name
    signal = get_time_signal(signal_name, experiment_name)
    response = response_ok

    timestamps = time_signal.timestamps
    signals = json.loads(time_signal.signal)
    signal_points = []

    for ts, sig in zip(timestamps, signals):
        if type(sig) is list:
            signal_points.append([ts] + sig)
        else:
            signal_points.append([ts, sig])
        # sig.append(ts)
        # signal_points.append(sig)

    data_connection.add_signal_points(signal['id'],
                                      signal_points)
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

    if req.time_signal != None and req.time_signal.name != '':
        try:
            send_time_signal(req.time_signal, req.experiment_name)
        except Exception as e:
            response = str(e)
            print e
            print 'Updating cache and retrying'
            map_key = (req.time_signal.name, req.experiment_name)
            if req.experiment_name in experiment_name_map:
                del experiment_name_map[req.experiment_name]
            if map_key in signal_name_map:
                del signal_name_map[map_key]
            send_time_signal(req.time_signal, req.experiment_name)

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
    response = data_connection.get_or_create_experiment(req.experiment)
    return RegisterExperimentResponse(response['id'])


def handle_register_setting(req):
    response = response_ok
    try:
        setting = data_connection.get_or_create_setting(req.key)
        data_connection.write_setting(req.key, req.value)
    except Exception as e:
        response = str(e)

    return RegisterSettingResponse(response)


def register_video(experiment, video_path):
    print "inside register video ..."
    try:
        setting = data_connection.get_or_create_setting('video', experiment)
        data_connection.write_setting(video_path, setting['id'])
        return response_ok
    except Exception as e:
        return str(e)


def handle_register_video(req):
    print "getting experiment ..."
    experiment = get_experiment(req.experiment)
    print "getting response ..."
    response = register_video(experiment, req.path)
    print response
    return RegisterVideoResponse(response)


def send_data_service():
    rospy.init_node('data_service', anonymous=True)
    s1 = rospy.Service('send_data', DataForward, handle_send_data)
    s2 = rospy.Service('retrieve_data', DataRetrieve, handle_retrieve_data)
    e1 = rospy.Service('register_experiment', RegisterExperiment, handle_register_experiment)
    v1 = rospy.Service('register_video', RegisterVideo, handle_register_video)

    settings_service = rospy.Service('register_setting', RegisterSetting, handle_register_setting)

    rospy.spin()


if __name__ == "__main__":
    send_data_service()
