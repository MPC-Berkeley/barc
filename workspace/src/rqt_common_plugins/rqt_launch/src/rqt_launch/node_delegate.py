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

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QStyledItemDelegate
import rospkg

from rqt_launch.node_widget import NodeWidget


class NodeDelegate(QStyledItemDelegate):
    __slots__ = ['status_label', 'respawn_toggle', 'spawn_count_label',
                 'launch_prefix_edit']

    def __init__(self, master_uri, rospack=None):
                #respawn_toggle, spawn_count_label, launch_prefix_edit):
        super(NodeDelegate, self).__init__()
        self._master_uri = master_uri

        self._rospack = rospack
        if rospack == None:
            self._rospack = rospkg.RosPack()

        self._nodewidget_dict = {}  # { QModelIndex : QWidget }

    def createEditor(self, parent, option, index):
        '''Overridden'''

        nodewidget = self._nodewidget_dict[index]
        #TODO: handle exception
        return nodewidget

    def setEditorData(self, spinBox, index):
        '''Overridden'''
        value = index.model().data(index, Qt.EditRole)

        spinBox.setValue(value)

    def setModelData(self, spinBox, model, index):
        '''Overridden'''
        spinBox.interpretText()
        value = spinBox.value()

        model.setData(index, value, Qt.EditRole)

    def updateEditorGeometry(self, editor, option, index):
        '''Overridden'''
        editor.setGeometry(option.rect)

    def create_node_widget(self, qindex, launch_config,
                           status_label):
        '''
        @type status_label: StatusIndicator
        '''
        nodewidget = NodeWidget(self._rospack,
                                self._master_uri, launch_config,
                                status_label)
        self._nodewidget_dict[qindex] = nodewidget
        return nodewidget

    def get_node_widget(self):
        return self._node_widget
