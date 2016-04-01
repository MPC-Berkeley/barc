# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Isaac Saito

import os

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QIcon, QLineEdit, QWidget
import rospy

from rqt_launch.name_surrogate import NamesSurrogate


class NodeWidget(QWidget):
    '''
    Works as a proxy between ROS Node
    (more in particular, roslaunch.nodeprocess) and GUI.
    '''

    __slots__ = ['_run_id', 'master_uri', 'config', '_process']

    def __init__(self, rospack, master_uri, launch_config,
                 label_status):
        '''
        @type launch_node: roslaunch.core.Node
        @type launch_config: roslaunch.core.Config
        @type label_status: StatusIndicator
        '''
        super(NodeWidget, self).__init__()
        self._rospack = rospack
        self._master_uri = master_uri
        self._launch_config = launch_config

        ui_file = os.path.join(self._rospack.get_path('rqt_launch'),
                               'resource', 'node_widget.ui')
        loadUi(ui_file, self)

        self.label_status = label_status  # Public
        # stop_button = QPushButton(self.style().standardIcon(
        #                                             QStyle.SP_MediaStop), "")
        self._respawn_toggle.setChecked(self._launch_config.respawn)
        self._lineEdit_launch_args = QLineEdit(
                                            self._launch_config.launch_prefix)

        rospy.logdebug('_proxy.conf.namespace={} launch_config={}'.format(
                      self._launch_config.namespace, self._launch_config.name))
        self._resolved_node_name = NamesSurrogate.ns_join(
                       self._launch_config.namespace, self._launch_config.name)
        self._label_nodename.setText(self._get_node_name())
        self._label_pkg_name.setText(self._launch_config.package)
        self._label_name_executable.setText(self._launch_config.type)

        self._icon_node_start = QIcon.fromTheme('media-playback-start')
        self._icon_node_stop = QIcon.fromTheme('media-playback-stop')
        self._icon_respawn_toggle = QIcon.fromTheme('view-refresh')

        self._pushbutton_start_stop_node.setIcon(self._icon_node_start)
        self._respawn_toggle.setIcon(self._icon_respawn_toggle)

        self._node_controller = None  # Connected in self.set_node_controller

    def _get_node_name(self):
        return self._resolved_node_name

    def connect_start_stop_button(self, slot):
        self._pushbutton_start_stop_node.toggled.connect(slot)

    def set_node_started(self, is_started=True):
        # If the button is not down yet
        is_node_running = self._node_controller.is_node_running()
        rospy.logdebug('NodeWidget.set_node_started running?={}'.format(
                                                            is_node_running))
        #if is_node_running:
        if is_started:
            #and self._pushbutton_start_stop_node.isDown():
            self._pushbutton_start_stop_node.setIcon(self._icon_node_start)
            self._pushbutton_start_stop_node.setDown(False)

        #elif not is_node_running:  # To START the node
        else:
            #and not self._pushbutton_start_stop_node.isDown():
            self._pushbutton_start_stop_node.setIcon(self._icon_node_stop)
            self._pushbutton_start_stop_node.setDown(True)

    def set_node_controller(self, node_controller):
        #TODO: Null check
        self._node_controller = node_controller
        #self._pushbutton_start_stop_node.pressed.connect(
        #                                self._node_controller.start_stop_slot)
