# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Surya Ambrose, Aldebaran
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

import rosgraph

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QScrollArea, QPushButton

class TopicSelection(QWidget):

    recordSettingsSelected = Signal(bool, list)

    def __init__(self):
        super(TopicSelection, self).__init__()
        master = rosgraph.Master('rqt_bag_recorder')
        self.setWindowTitle("Select the topics you want to record")
        self.resize(500, 700)

        self.topic_list = []
        self.selected_topics = []
        self.items_list = []

        self.area = QScrollArea(self)
        self.main_widget = QWidget(self.area)
        self.ok_button = QPushButton("Record", self)
        self.ok_button.clicked.connect(self.onButtonClicked)
        self.ok_button.setEnabled(False)

        self.main_vlayout = QVBoxLayout(self)
        self.main_vlayout.addWidget(self.area)
        self.main_vlayout.addWidget(self.ok_button)
        self.setLayout(self.main_vlayout)

        self.selection_vlayout = QVBoxLayout(self)
        self.item_all = QCheckBox("All", self)
        self.item_all.stateChanged.connect(self.updateList)
        self.selection_vlayout.addWidget(self.item_all)
        topic_data_list = master.getPublishedTopics('')
        topic_data_list.sort()
        for topic, datatype in topic_data_list:
            self.addCheckBox(topic)

        self.main_widget.setLayout(self.selection_vlayout)

        self.area.setWidget(self.main_widget)
        self.show()

    def addCheckBox(self, topic):
        self.topic_list.append(topic)
        item = QCheckBox(topic, self)
        item.stateChanged.connect(lambda x: self.updateList(x,topic))
        self.items_list.append(item)
        self.selection_vlayout.addWidget(item)


    def updateList(self, state, topic = None):
        if topic is None: # The "All" checkbox was checked / unchecked
            if state == Qt.Checked:
                self.item_all.setTristate(False)
                for item in self.items_list:
                    if item.checkState() == Qt.Unchecked:
                        item.setCheckState(Qt.Checked)
            elif state == Qt.Unchecked:
                self.item_all.setTristate(False)
                for item in self.items_list:
                    if item.checkState() == Qt.Checked:
                        item.setCheckState(Qt.Unchecked)
        else:
            if state == Qt.Checked:
                self.selected_topics.append(topic)
            else:
                self.selected_topics.remove(topic)
                if self.item_all.checkState()==Qt.Checked:
                    self.item_all.setCheckState(Qt.PartiallyChecked)

        if self.selected_topics == []:
            self.ok_button.setEnabled(False)
        else:
            self.ok_button.setEnabled(True)

    def onButtonClicked(self):
        self.close()
        self.recordSettingsSelected.emit((self.item_all.checkState() == Qt.Checked), self.selected_topics)
