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
# Author: Isaac Saito, Ze'ev Klapow

import threading
import time

import rospy


class ParamUpdater(threading.Thread):
    '''
    Using dynamic_reconfigure that is passed in __init__, this thread updates
    the Dynamic Reconfigure server's value.

    This class works for a single element in a single parameter.
    '''

    #TODO: Modify variable names to the ones that's more intuitive.

    def __init__(self, reconf):
        """
        :type reconf: dynamic_reconfigure
        """
        super(ParamUpdater, self).__init__()
        self.setDaemon(True)

        self._reconf = reconf
        self._condition_variable = threading.Condition()
        self._configs_pending = {}
        self._timestamp_last_pending = None
        self._stop_flag = False

    def run(self):
        _timestamp_last_commit = None

        rospy.logdebug(' ParamUpdater started')

        while not self._stop_flag:
            if _timestamp_last_commit >= self._timestamp_last_pending:
                    with self._condition_variable:
                        rospy.logdebug(' ParamUpdater loop 1.1')
                        self._condition_variable.wait()
                        rospy.logdebug(' ParamUpdater loop 1.2')
            rospy.logdebug(' ParamUpdater loop 2')

            if self._stop_flag:
                return

            _timestamp_last_commit = time.time()
            configs_tobe_updated = self._configs_pending.copy()
            self._configs_pending = {}

            rospy.logdebug('  run last_commit={}, last_pend={}'.format(
                         _timestamp_last_commit, self._timestamp_last_pending))

            try:
                self._reconf.update_configuration(configs_tobe_updated)
            except rospy.ServiceException as ex:
                rospy.logdebug('Could not update configs due to {}'.format(
                                                                     ex.value))
            except Exception as exc:
                raise exc

    def update(self, config):
        with self._condition_variable:
            for name, value in config.items():
                self._configs_pending[name] = value

            self._timestamp_last_pending = time.time()

            self._condition_variable.notify()

    def stop(self):
        self._stop_flag = True
        with self._condition_variable:
            self._condition_variable.notify()
