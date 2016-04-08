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


rosbag_dir = '.'


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
        self.record_started = False

        self.setup_topics_list()
        self.p = None



    def setup_topics_list(self):
        topics = ['imu_data', 'enc_data', 'ecu_cmd']

        for t in topics:
            item = QListWidgetItem(t)
            item.setCheckState(True)

            self._widget.listview_topics.addItem(item)


    def _handle_record(self, checked):
        if not self._widget.experiment_name.text():
            msg = QMessageBox()
            msg.setText('Need an experiment name!')
            msg.exec_()
            return

        if self.record_started:
            self.record_started = False
            self.stop_record_data()
            self._widget.label_experiment.setText('Experiment name')
            self._widget.pushbutton_record.setText('Start Recording')
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

        command += ' -O %s' % self._widget.experiment_name.text()

        self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=rosbag_dir)


    # TODO: Do this in the background thread!!!
    def stop_record_data(self):
        if not self.p:
            return

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


    def upload_data(self):
        rosbag_file = os.path.abspath(rosbag_dir + '/' + self._widget.experiment_name.text() + '.bag')

        date = time.strftime("%Y.%m.%d")
        experiment = self._widget.experiment_name.text()

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
        vars_list = ['roll', 'pitch', 'yaw', 'a_x', 'a_y', 'a_z', 'w_x', 'w_y', 'w_z', 'n_FL', 'n_FR',
                     'motor_pwm', 'servo_pwm']

        signal_dict = dict()

        for v in vars_list:
            signal_dict[v] = []

        for msg in msgs:
            if topic == 'imu_data':
                (roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = msg.value

            # TODO: Encoder data is raw
            if topic == 'enc_data':
                n_FL = msg.x
                n_FR = msg.y

            if topic == 'ecu_cmd':
                motor_pwm = msg.x
                servo_pwm = msg.y
                d_f = msg.z

            # Python introspection
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
