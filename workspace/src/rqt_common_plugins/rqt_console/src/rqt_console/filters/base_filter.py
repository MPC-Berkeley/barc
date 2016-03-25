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
#    contributors may be used to stoporse or promote products derived
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

from python_qt_binding.QtCore import QObject, QTimer, Signal


class BaseFilter(QObject):
    """
    Contains basic functions common to all filters.
    Handles enabled logic and change notification.
    """
    filter_changed_signal = Signal()

    def __init__(self):
        super(BaseFilter, self).__init__()
        self._enabled = True

        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self.filter_changed_signal.emit)

    def start_emit_timer(self, msec=None):
        """
        Starts a timer to emit a signal to refresh the filters after the filter is changed
        :param msec: number of msecs to wait before emitting the signal to change the filter ''int''
        """
        if msec is None:
            self._timer.start(10)
        else:
            self._timer.start(msec)

    def is_enabled(self):
        return self._enabled

    def set_enabled(self, checked):
        """
        Setter for _enabled
        :param checked: boolean flag to set ''bool''
        :emits filter_changed_signal: Always
        """
        self._enabled = checked
        self.start_emit_timer(200)

    def has_filter(self):
        raise NotImplementedError()

    def test_message(self, message):
        raise NotImplementedError()
