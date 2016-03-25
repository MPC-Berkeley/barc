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

from rosgraph_msgs.msg import Log

from python_qt_binding.QtCore import QCoreApplication, QDateTime, QObject


class Message(QObject):

    DEBUG = 1
    INFO = 2
    WARN = 4
    ERROR = 8
    FATAL = 16

    SEVERITY_LABELS = {
        DEBUG: QCoreApplication.translate('Message', 'Debug'),
        INFO: QCoreApplication.translate('Message', 'Info'),
        WARN: QCoreApplication.translate('Message', 'Warn'),
        ERROR: QCoreApplication.translate('Message', 'Error'),
        FATAL: QCoreApplication.translate('Message', 'Fatal'),
    }

    _next_id = 1

    def __init__(self):
        super(Message, self).__init__()
        self.id = Message._next_id
        Message._next_id += 1

        self.message = None
        self.severity = None
        self.node = None
        self.__stamp = (None, None)
        self.topics = []
        self.location = None

        self._stamp_compare = None
        self._stamp_qdatetime = None

        self._stamp_format = None
        self._stamp_string = None

        self.highlighted = True

    def _get_stamp(self):
        return self.__stamp

    def _set_stamp(self, stamp):
        """
        Update the string representation of the timestamp.
        :param stamp: a tuple containing seconds and nano seconds
        """
        assert len(stamp) == 2
        self.__stamp = stamp
        if None not in self.__stamp:
            # shortest string representation to compare stamps
            # floats do not provide enough precision
            self._stamp_compare = '%08x%08x' % (self.__stamp[0], self.__stamp[1])
        else:
            self._stamp_compare = None
        self._stamp_qdatetime = self._get_stamp_as_qdatetime(self.__stamp)
        if self._stamp_format:
            s = self._stamp_qdatetime.toString(self._stamp_format)
            if 'ZZZ' in s:
                s = s.replace('ZZZ', str(self.__stamp[1]).zfill(9))
            self._stamp_string = s

    stamp = property(_get_stamp, _set_stamp)

    def get_stamp_for_compare(self):
        return self._stamp_compare

    def get_stamp_as_qdatetime(self):
        return self._stamp_qdatetime

    def _get_stamp_as_qdatetime(self, stamp):
        if None in self.__stamp:
            return None
        dt = QDateTime()
        dt.setTime_t(stamp[0])
        dt.addMSecs(int(float(stamp[1]) / 10**6))
        return dt

    def get_stamp_string(self):
        return self._stamp_string

    def set_stamp_format(self, format):
        self._stamp_format = format
        if None not in self.__stamp:
            self.stamp = self.__stamp

    def pretty_print(self):
        text = self.tr('Node: ') + self.node + '\n'
        text += self.tr('Time: ') + self.get_stamp_string() + '\n'
        text += self.tr('Severity: ') + Message.SEVERITY_LABELS[self.severity] + '\n'
        text += self.tr('Published Topics: ') + ', '.join(self.topics) + '\n'
        text += '\n' + self.message + '\n'
        text += '\n' + 'Location:'
        text += '\n' + self.location + '\n\n'
        text += '-' * 100 + '\n\n'

        return text
