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

from python_qt_binding.QtCore import QDateTime
from .base_filter import BaseFilter


class TimeFilter(BaseFilter):
    """
    Contains filter logic for a time filter.
    If _stop_time_enabled is true then the message's time value must be between the dates provided
    to be considered a match
    If _stop_time_enabled is false then the time must simply be after _start_time
    """

    def __init__(self):
        super(TimeFilter, self).__init__()
        self._start_time = QDateTime()
        self._stop_time = QDateTime()
        self._stop_time_enabled = True

    def set_start_time(self, time):
        """
        Setter for _start_time
        :param time" start datetime for filter ''QDateTime''
        :emits filter_changed_signal: If _enabled is true
        """
        self._start_time = time
        if self.is_enabled():
            self.start_emit_timer()

    def set_stop_time(self, time):
        """
        Setter for _stop_time
        :param time" stop datetime for filter ''QDateTime''
        :emits filter_changed_signal: If _enabled is true
        """
        self._stop_time = time
        if self.is_enabled():
            self.start_emit_timer()

    def set_stop_time_enabled(self, checked):
        """
        Setter for _stop_time_enabled
        :param checked" boolean flag to set ''bool''
        :emits filter_changed_signal: If _enabled is true
        """
        self._stop_time_enabled = checked
        if self.is_enabled():
            self.start_emit_timer()

    def has_filter(self):
        return True

    def test_message(self, message):
        """
        Tests if the message matches the filter.
        If _stop_time_enabled is true then the message's time value must be between the dates provided
        to be considered a match
        If _stop_time_enabled is false then the time must simply be after _start_time
        :param message: the message to be tested against the filters, ''Message''
        :returns: True if the message matches, ''bool''
        """
        if not self.is_enabled():
            return False
        message_time = message.get_stamp_as_qdatetime()
        if message_time < self._start_time:
            return False
        if self._stop_time_enabled and self._stop_time < message_time:
            return False
        return True
