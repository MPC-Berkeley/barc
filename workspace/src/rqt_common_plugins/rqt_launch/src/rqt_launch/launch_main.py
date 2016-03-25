# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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

import sys

import rospy
from rqt_launch.launch_widget import LaunchWidget
from rqt_py_common.plugin_container_widget import PluginContainerWidget
from rqt_py_common.rqt_roscomm_util import RqtRoscommUtil


class LaunchMain(object):
    def __init__(self, plugin_context):
        super(LaunchMain, self).__init__()
        self._plugin_context = plugin_context

        self._main_launch_widget = LaunchWidget(self)
        self._mainwidget = PluginContainerWidget(self._main_launch_widget,
                                                 True, False)

        self._run_id = None
        self._node_controllers = []
        self._config = None

        #RqtRoscommUtil.load_parameters(self._config, '/rqt_launch')

    def get_widget(self):
        return self._mainwidget

    def set_node_controllers(self, node_controllers):
        self._node_controllers = node_controllers

    def set_config(self, config):
        self._config = config

    def start_all(self):
        '''
        Checks nodes that's set (via self.set_node_controllers) one by one and
        starts one if each node is not running.
        Then disable START ALL button and enable STOP ALL button.
        '''

        if self._config:
            for k in self._config.params.keys():
                rospy.set_param(k, self._config.params[k])

        for n in self._node_controllers:
            if not n.is_node_running():
                n.start(restart=False)

        # Disable START ALL button.
        self._main_launch_widget._pushbutton_start_all.setEnabled(False)
        self._main_launch_widget._pushbutton_stop_all.setEnabled(True)

    def stop_all(self):
        '''
        Checks nodes that's set (via self.set_node_controllers) one by one and
        stops one if each node is running.
        Then enable START ALL button and disable STOP ALL button.
        '''

        for n in self._node_controllers:
            if n.is_node_running():
                n.stop()

        # Disable STOP ALL button.
        self._main_launch_widget._pushbutton_start_all.setEnabled(True)
        self._main_launch_widget._pushbutton_stop_all.setEnabled(False)

    def check_process_statuses(self):
        for n in self._node_controllers:
            n.check_process_status()

    def shutdown(self):
        rospy.logdebug('Launchmain.shutdown')
        self.stop_all()

    def save_settings(self, plugin_settings, instance_settings):
        self._mainwidget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._mainwidget.restore_settings(plugin_settings, instance_settings)


if __name__ == '__main__':
    # main should be used only for debug purpose.
    # This launches this QWidget as a standalone rqt gui.
    from rqt_gui.main import Main
    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_launch'))
