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

import os

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QDialog

from rqt_logger_level.logger_level_widget import LoggerLevelWidget
from rqt_logger_level.logger_level_service_caller import LoggerLevelServiceCaller


class ConsoleSettingsDialog(QDialog):
    """
    Dialog to change the subscribed topic and alter the message buffer size.
    """
    def __init__(self, topics, rospack):
        """
        :param topics: list of topics to allow switching, ''list of string''
        :param limit: displayed in the message buffer size spin box, ''int''
        """
        super(ConsoleSettingsDialog, self).__init__()
        ui_file = os.path.join(rospack.get_path('rqt_console'), 'resource', 'console_settings_dialog.ui')
        loadUi(ui_file, self)
        for topic in topics:
            self.topic_combo.addItem(topic[0] + ' (' + topic[1] + ')', topic[0])

        self._service_caller = LoggerLevelServiceCaller()
        self._logger_widget = LoggerLevelWidget(self._service_caller)
        self.levelsLayout.addWidget(self._logger_widget)
        self.adjustSize()

    def query(self, topic, buffer_size):
        index = self.topic_combo.findData(topic)
        if index != -1:
            self.topic_combo.setCurrentIndex(index)
        self.buffer_size_spin.setValue(buffer_size)
        ok = self.exec_()
        if ok == 1:
            index = self.topic_combo.currentIndex()
            if index != -1:
                topic = self.topic_combo.itemData(index)
            buffer_size = self.buffer_size_spin.value()
        return topic, buffer_size
