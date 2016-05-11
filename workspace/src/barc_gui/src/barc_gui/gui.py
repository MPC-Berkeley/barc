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
# ---------------------------------------------------------------------------

import roslib
roslib.load_manifest('barc_gui')

import subprocess, os, signal, psutil
import json

import rospy
import rosbag

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

import std_msgs.msg

from PyQt4 import QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import time

from data_service.srv import *
from data_service.msg import *

from threading import *


rosbag_dir = '.'
video_dir = '.'


class UploadThread(Thread):

    def __init__(self, plugin_ob):
        Thread.__init__(self)
        self.plugin_ob = plugin_ob


    def run(self):
        self.plugin_ob.record_data_thread()


class MyGUI(Plugin):

    def __init__(self, context):
        super(MyGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyGUI')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'BARC_experiment.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyGUIUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.pushbutton_record.clicked[bool].connect(self._handle_record)
        self._widget.pushbutton_record_video.clicked[bool].connect(self._handle_record_video)
        self.record_started = False
        self.record_video_started = False

        self.setup_topics_list()
        self.p = None


    def get_experiment_name(self):
        return self._widget.experiment_name.text().replace(' ', '_')


    def setup_topics_list(self):
        topics = ['imu', 'encoder', 'ecu', 'ultrasound']

        for t in topics:
            item = QListWidgetItem(t)
            item.setCheckState(True)

            self._widget.listview_topics.addItem(item)


    def _handle_record_video(self, checked):
        if not self.get_experiment_name():
            msg = QMessageBox()
            msg.setText('Need an experiment name!')
            msg.exec_()
            return

        if self.record_video_started:
            self.record_video_started = False
            self.stop_record_video()
            # self._widget.label_experiment.setText('Experiment name')
            # self._widget.pushbutton_record.setText('Start Recording')
        else:
            self.record_video_started = True
            self._widget.pushbutton_record_video.setText('Stop Recording Video')
            # self._widget.label_experiment.setText('Recording...')
            self.start_record_video()


    def start_record_video(self):

        print 'Recording video...'

        command = 'rosrun image_view video_recorder image:=/image_raw _max_depth_range:=0'

        self.p_video = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=video_dir)


    def stop_record_video(self):
        print 'Stopping record video'

        if not self.p_video:
            print 'Not stopping record video because p_video NONE'
            return

        command = 'rosnode list'
        out = subprocess.check_output(command, shell=True)
        for li in out.split('\n'):
            if 'video_recorder' in li:
                command_kill = 'rosnode kill %s' % li
                ps = subprocess.Popen(command_kill, stdin=subprocess.PIPE, shell=True)
                ps.communicate()

        command = 'mv output.avi %s.avi' % self.get_experiment_name()
        subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=video_dir)

        self._widget.pushbutton_record.setText('Start Recording Video')


    def _handle_record(self, checked):
        if not self.get_experiment_name():
            msg = QMessageBox()
            msg.setText('Need an experiment name!')
            msg.exec_()
            return

        if self.record_started:
            self.record_started = False
            self.stop_record_data()
            # self._widget.label_experiment.setText('Experiment name')
            # self._widget.pushbutton_record.setText('Start Recording')
        else:
            self.record_started = True
            self._widget.pushbutton_record.setText('Stop Recording')
            self._widget.label_experiment.setText('Recording...')
            self.time = time.time()
            self.start_record_data()


    def start_record_data(self):
        record_topics = []
        for index in range(self._widget.listview_topics.count()):
            item = self._widget.listview_topics.item(index)
            if item.checkState():
                record_topics.append(item.text())

        command = 'rosbag record '

        for topic in record_topics:
            command += topic + ' '

        command += ' -O %s' % self.get_experiment_name()

        self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=rosbag_dir)


    def stop_record_data(self):
        if not self.p:
            return

        uploader = UploadThread(self)
        uploader.start()


    def record_data_thread(self):

        rospy.wait_for_service('send_data')
        self.send_data = rospy.ServiceProxy('send_data', DataForward, persistent=True)
        command = 'rosnode list'
        out = subprocess.check_output(command, shell=True)

        for li in out.split('\n'):
            if 'record' in li:
                command_kill = 'rosnode kill %s' % li
                ps = subprocess.Popen(command_kill, stdin=subprocess.PIPE, shell=True)
                ps.communicate()

        self._widget.label_experiment.setText('Uploading data...')
        self.upload_data()
        self._widget.label_experiment.setText('Experiment name')
        self._widget.pushbutton_record.setText('Start Recording')


    def upload_data(self):
        print 'Uploading data!!!!'
        rosbag_file = os.path.abspath(rosbag_dir + '/' + self.get_experiment_name() + '.bag')

        date = time.strftime("%Y.%m.%d")
        experiment = self.get_experiment_name()

        experiment += '_' + date + '_' + time.strftime('%H.%M')

        for i in range(100):
            if os.path.isfile(rosbag_file):
                break
            time.sleep(1.0)

        if not os.path.isfile(rosbag_file):
            return

        convert_to_time = 1/(10.0**(9))

        bag = rosbag.Bag(rosbag_file)

        chunk_size = 50
        chunk_dict = dict()
        chunk_msg = dict()
        chunk_ts = dict()

        for topic, msg, t in bag.read_messages():
            ts = t.nsecs * convert_to_time
            ts += (t.secs - self.time)

            if topic not in chunk_dict:
                chunk_dict[topic] = 1

            if topic not in chunk_msg:
                chunk_msg[topic] = []

            if topic not in chunk_ts:
                chunk_ts[topic] = []

            chunk_dict[topic] += 1
            chunk_msg[topic].append(msg)
            chunk_ts[topic].append(ts)

            for k, v in chunk_dict.items():
                if v > chunk_size:
                    self.upload_message(k, chunk_msg[k], chunk_ts[k], experiment)
                    del chunk_msg[k]
                    del chunk_dict[k]
                    del chunk_ts[k]

        for k, v in chunk_dict.items():
            self.upload_message(k, chunk_msg[k], chunk_ts[k], experiment)

        os.remove(rosbag_file)


    def upload_message(self, topic, msgs, tss, experiment):
        vars_list = ['roll', 'pitch', 'yaw',
                      'roll_rate', 'pitch_rate', 'yaw_rate',
                     'acc_x', 'acc_y', 'acc_z',
                     'encoder_FL', 'encoder_FR','encoder_BL','encoder_BR',
                     'motor_pwm', 'servo_pwm',
                     'ultrasound_front','ultrasound_back','ultrasound_left','ultrasound_right']

        signal_dict = dict()

        for v in vars_list:
            signal_dict[v] = []

        for msg in msgs:
            # Inertia measurement unit
            if topic == 'imu':
                (roll, pitch, yaw, acc_x, acc_y, acc_z, roll_rate, pitch_rate, yaw_rate) = msg.value

            # Encoder
            if topic == 'encoder':
                encoder_FL = msg.FL
                encoder_FR = msg.FR
                encoder_BL = msg.BL
                encoder_BR = msg.BR

            if topic == 'ultrasound':
                ultrasound_front = msg.front
                ultrasound_back = msg.back
                ultrasound_left = msg.left
                ultrasound_right = msg.right

            # Electronic control unit
            if topic == 'ecu':
                motor_pwm = msg.motor_pwm
                servo_pwm = msg.servo_pwm

            # Python introspection from list 'vars_list'
            for v in vars_list:
                if v in dict(globals(), **locals()):
                    signal_dict[v].append(dict(globals(), **locals())[v])

        for v in vars_list:
            if signal_dict[v]:
                time_signal = TimeSignal()
                time_signal.name = v
                time_signal.timestamps = tss
                time_signal.signal = json.dumps(signal_dict[v])
                try:
                    self.send_data(time_signal, None, experiment)
                except Exception as e:
                    pass
