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
import rospkg
import rospy

from python_qt_binding.QtCore import QMutex, QMutexLocker, QTimer

from qt_gui.plugin import Plugin

from .console_settings_dialog import ConsoleSettingsDialog
from .console_widget import ConsoleWidget
from .message import Message
from .message_data_model import MessageDataModel
from .message_proxy_model import MessageProxyModel


class Console(Plugin):
    """
    rqt_console plugin's main class. Handles communication with ros_gui and contains
    callbacks to handle incoming message
    """
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(Console, self).__init__(context)
        self.setObjectName('Console')

        self._rospack = rospkg.RosPack()

        self._model = MessageDataModel()
        self._proxy_model = MessageProxyModel()
        self._proxy_model.setSourceModel(self._model)

        self._widget = ConsoleWidget(self._proxy_model, self._rospack)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # queue to store incoming data which get flushed periodically to the model
        # required since QSortProxyModel can not handle a high insert rate
        self._message_queue = []
        self._mutex = QMutex()
        self._timer = QTimer()
        self._timer.timeout.connect(self.insert_messages)
        self._timer.start(100)

        self._subscriber = None
        self._topic = '/rosout_agg'
        self._subscribe(self._topic)

    def queue_message(self, log_msg):
        """
        Callback for adding an incomming message to the queue.
        """
        if not self._widget._paused:
            msg = Console.convert_rosgraph_log_message(log_msg)
            with QMutexLocker(self._mutex):
                self._message_queue.append(msg)

    @staticmethod
    def convert_rosgraph_log_message(log_msg):
        msg = Message()
        msg.set_stamp_format('hh:mm:ss.ZZZ (yyyy-MM-dd)')
        msg.message = log_msg.msg
        msg.severity = log_msg.level
        msg.node = log_msg.name
        msg.stamp = (log_msg.header.stamp.secs, log_msg.header.stamp.nsecs)
        msg.topics = sorted(log_msg.topics)
        msg.location = log_msg.file + ':' + log_msg.function + ':' + str(log_msg.line)
        return msg

    def insert_messages(self):
        """
        Callback for flushing incoming Log messages from the queue to the model.
        """
        with QMutexLocker(self._mutex):
            msgs = self._message_queue
            self._message_queue = []
        if msgs:
            self._model.insert_rows(msgs)

    def shutdown_plugin(self):
        self._subscriber.unregister()
        self._timer.stop()
        self._widget.cleanup_browsers_on_close()

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    def trigger_configuration(self):
        topics = [t for t in rospy.get_published_topics() if t[1] == 'rosgraph_msgs/Log']
        topics.sort(key=lambda tup: tup[0])
        dialog = ConsoleSettingsDialog(topics, self._rospack)
        (topic, message_limit) = dialog.query(self._topic, self._model.get_message_limit())
        if topic != self._topic:
            self._subscribe(topic)
        if message_limit != self._model.get_message_limit():
            self._model.set_message_limit(message_limit)

    def _subscribe(self, topic):
        if self._subscriber:
            self._subscriber.unregister()
        self._subscriber = rospy.Subscriber(topic, Log, self.queue_message)
        self._currenttopic = topic
