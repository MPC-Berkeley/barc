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

import time
import threading

from roslaunch import node_args, nodeprocess
import rospy


class Polling(threading.Thread):
    def __init__(self, parent):
        super(Polling, self).__init__()
        self._parent = parent

    def run(self):
        while True:
            rospy.logdebug('Proc={} Died? {}'.format(
                                               self._parent.get_proc_name(),
                                               self._parent.has_died()))
            time.sleep(1.0)


class NodeProxy(object):
    '''
    Works as a proxy between ROS Node
    (more in particular, roslaunch.nodeprocess) & GUI.
    '''

    __slots__ = ['_run_id', 'master_uri', 'config', '_process']

    def __init__(self, run_id, master_uri, config):
        self._run_id = run_id
        self.master_uri = master_uri
        self.config = config

        # @type: roslaunch.nodeprocess.LocalProcess
        self._process = self.recreate_process()

    # LocalProcess.is_alive() does not do what you would expect
    def is_running(self):
        rospy.logdebug('BEFORE started={}, stopped={} alive={}'.format(
                                                     self._process.started,
                                                     self._process.stopped,
                                                     self._process.is_alive()))
        return self._process.started and not self._process.stopped
                #and self._process.is_alive()
                #  is_alive() returns False once nodeprocess was stopped.

    def has_died(self):
        rospy.logdebug('Proc={} started={}, stopped={}, is_alive={}'.format(
            self.get_proc_name(), self._process.started, self._process.stopped,
            self._process.is_alive()))

        return (self._process.started and not self._process.stopped
                and not self._process.is_alive())

    def recreate_process(self):
        '''
        Create and set roslaunch.nodeprocess.LocalProcess to member variable.
        @rtype: roslaunch.nodeprocess.LocalProcess
        '''
        _local_process = nodeprocess.LocalProcess
        try:
            _local_process = nodeprocess.create_node_process(
                                    self._run_id, self.config, self.master_uri)
        except node_args.NodeParamsException as e:
            rospy.logerr('roslaunch failed to load the node. {}'.format(
                                                                   e.message))
            #TODO: Show msg on GUI that the node wasn't loaded.

        return _local_process

    def start_process(self):
        #TODO: add null exception check for _process
        self._process.start()

    def stop_process(self):
        #TODO: add null exception check for _process

        try:
            self._process.stop()
        except Exception as e:
            #TODO: Change roslaunch to raise exception
            rospy.logdebug('Proxy stop_process exception={}'.format(e.message))

    def get_spawn_count(self):
        return self._process.spawn_count

    def get_proc_name(self):
        return self._process.name

    def get_proc_exit_code(self):
        return self._process.exit_code
