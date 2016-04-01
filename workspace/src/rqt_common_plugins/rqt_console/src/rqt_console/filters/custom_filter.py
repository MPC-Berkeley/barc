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

from .base_filter import BaseFilter
from .message_filter import MessageFilter
from .node_filter import NodeFilter
from .severity_filter import SeverityFilter
from .topic_filter import TopicFilter


class CustomFilter(BaseFilter):
    """
    Contains filter logic for the custom filter which allows message, severity,
    node and topic filtering simultaniously. All of these filters must match
    together (if they are used) or the custom filter does not match.
    """

    def __init__(self):
        super(CustomFilter, self).__init__()

        self._message = MessageFilter()
        self._severity = SeverityFilter()
        self._node = NodeFilter()
        self._topic = TopicFilter()

        self._all_filters = [self._message, self._severity, self._node, self._topic]
        for f in self._all_filters:
            f.filter_changed_signal.connect(self._relay_signal)

    def set_enabled(self, checked):
        """
        :signal: emits filter_changed_signal
        :param checked: enables the filters if checked is True''bool''
        """
        for f in self._all_filters:
            f.set_enabled(checked)
        super(CustomFilter, self).set_enabled(checked)

    def _relay_signal(self):
        """
        Passes any signals emitted by the child filters along
        """
        self.start_emit_timer(1)

    def has_filter(self):
        for f in self._all_filters:
            if f.has_filter():
                return True
        return False

    def test_message(self, message):
        """
        Tests if the message matches the filter.
        :param message: the message to be tested against the filters, ''Message''
        :returns: True if the message matches all child filters, ''bool''
        """
        if not self.is_enabled():
            return False
        # if non of the subfilters contains any input the custom filter does not match
        if not self.has_filter():
            return False
        # the custom filter matches when all subfilters which contain input match
        all_filters = [not f.has_filter() or f.test_message(message) for f in self._all_filters]
        return False not in all_filters
