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

import threading

from python_qt_binding.QtCore import QCoreApplication, QEvent
from python_qt_binding.QtCore import qWarning


class ListenerEvent(QEvent):
    def __init__(self, data):
        super(ListenerEvent, self).__init__(1024)  # userdefined event constant
        self.data = data


class MessageListenerThread(threading.Thread):
    """
    Waits for new messages loaded on the given topic, then calls the message listener.
    One thread per listener, topic pair.
    """
    def __init__(self, timeline, topic, listener):
        threading.Thread.__init__(self)

        self.timeline = timeline
        self.topic = topic
        self.listener = listener
        self.bag_msg_data = None
        self._stop_flag = False
        self.setDaemon(True)
        self.start()

    def run(self):
        """
        Thread body. loops and notifies the listener of new messages
        """
        while not self._stop_flag:
            # Wait for a new message
            cv = self.timeline._messages_cvs[self.topic]
            with cv:
                while (self.topic not in self.timeline._messages) or (self.bag_msg_data == self.timeline._messages[self.topic]):
                    cv.wait()
                    if self._stop_flag:
                        return
                bag_msg_data = self.timeline._messages[self.topic]
            # View the message
            self.bag_msg_data = bag_msg_data
            try:
                event = ListenerEvent(bag_msg_data)
                QCoreApplication.postEvent(self.listener, event)
            except Exception, ex:
                qWarning('Error notifying listener %s: %s' % (type(self.listener), str(ex)))

    def stop(self):
        self._stop_flag = True
        cv = self.timeline._messages_cvs[self.topic]
        with cv:
            cv.notify_all()
