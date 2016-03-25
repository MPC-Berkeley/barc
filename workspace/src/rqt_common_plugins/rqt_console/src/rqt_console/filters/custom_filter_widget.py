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
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QPalette, QWidget

from rqt_py_common.ini_helper import pack, unpack


class CustomFilterWidget(QWidget):
    def __init__(self, parentfilter, rospack, item_providers):
        super(CustomFilterWidget, self).__init__()
        ui_file = os.path.join(rospack.get_path('rqt_console'), 'resource/filters', 'custom_filter_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('CustomFilterWidget')
        self._parentfilter = parentfilter  # When data is changed it is stored in the parent filter

        # keep color for highlighted items even when not active
        for list_widget in [self.severity_list, self.node_list, self.topic_list]:
            active_color = list_widget.palette().brush(QPalette.Active, QPalette.Highlight).color().name()
            list_widget.setStyleSheet('QListWidget:item:selected:!active { background: %s; }' % active_color)

        # Text Filter Initialization
        self.text_edit.textChanged.connect(self.handle_text_changed)
        self.regex_check_box.clicked[bool].connect(self.handle_regex_clicked)
        self.handle_text_changed()

        # Severity Filter Initialization
        self.severity_list.itemSelectionChanged.connect(self.handle_severity_item_changed)
        new_items = item_providers[0]()
        for key in sorted(new_items.keys()):
            item = new_items[key]
            self.severity_list.addItem(item)
            self.severity_list.item(self.severity_list.count() - 1).setData(Qt.UserRole, key)

        # Node Filter Initialization
        self._node_list_populate_function = item_providers[1]
        self.node_list.itemSelectionChanged.connect(self.handle_node_item_changed)

        # Topic Filter Initialization
        self._topic_list_populate_function = item_providers[2]
        self.topic_list.itemSelectionChanged.connect(self.handle_topic_item_changed)

        self.repopulate()

    def handle_node_item_changed(self):
        self._parentfilter._node.set_selected_items(self.node_list.selectedItems())

    def handle_topic_item_changed(self):
        self._parentfilter._topic.set_selected_items(self.topic_list.selectedItems())

    def handle_severity_item_changed(self):
        self._parentfilter._severity.set_selected_items(self.severity_list.selectedItems())

    def handle_text_changed(self):
        self._parentfilter._message.set_text(self.text_edit.text())

    def handle_regex_clicked(self, clicked):
        self._parentfilter._message.set_regex(clicked)

    def repopulate(self):
        """
        Repopulates the display widgets based on the function arguments passed
        in during initialization
        """
        newset = self._topic_list_populate_function()
        for item in newset:
            if len(self.topic_list.findItems(item, Qt.MatchExactly)) == 0:
                self._add_item(self.topic_list, item)

        newset = self._node_list_populate_function()
        for item in newset:
            if len(self.node_list.findItems(item, Qt.MatchExactly)) == 0:
                self._add_item(self.node_list, item)

    def _add_item(self, list_widget, item):
        """
        Insert item in alphabetical order.
        """
        for i in range(list_widget.count()):
            if item <= list_widget.item(i).text():
                list_widget.insertItem(i, item)
                return
        list_widget.addItem(item)

    def save_settings(self, settings):
        """
        Saves the settings for this filter to an ini file.
        :param settings: used to write the settings to an ini file ''qt_gui.settings.Settings''
        """
        settings.set_value('text', self._parentfilter._message._text)
        settings.set_value('regex', self._parentfilter._message._regex)

        settings.set_value('severityitemlist', pack(self._parentfilter._severity._selected_items))

        settings.set_value('topicitemlist', pack(self._parentfilter._topic._selected_items))

        settings.set_value('nodeitemlist', pack(self._parentfilter._node._selected_items))

        return

    def restore_settings(self, settings):
        """
        Restores the settings for this filter from an ini file.
        :param settings: used to extract the settings from an ini file ''qt_gui.settings.Settings''
        """
        text = settings.value('text', '')
        self.text_edit.setText(text)
        self.handle_text_changed()

        regex = settings.value('regex') in [True, 'true']
        self.regex_check_box.setChecked(regex)
        self.handle_regex_clicked(regex)

        #Restore Severities
        for index in range(self.severity_list.count()):
            self.severity_list.item(index).setSelected(False)
        severity_item_list = unpack(settings.value('severityitemlist'))
        for item in severity_item_list:
            items = self.severity_list.findItems(item, Qt.MatchExactly)
            for item in items:
                item.setSelected(True)
        self.handle_severity_item_changed()

        #Restore Topics
        for index in range(self.topic_list.count()):
            self.topic_list.item(index).setSelected(False)
        topic_item_list = unpack(settings.value('topicitemlist'))
        for item in topic_item_list:
            if len(self.topic_list.findItems(item, Qt.MatchExactly)) == 0:
                self.topic_list.addItem(item)
            items = self.topic_list.findItems(item, Qt.MatchExactly)
            for item in items:
                item.setSelected(True)
        self.handle_topic_item_changed()

        #Restore Nodes
        for index in range(self.node_list.count()):
            self.node_list.item(index).setSelected(False)
        node_item_list = unpack(settings.value('nodeitemlist'))
        for item in node_item_list:
            if len(self.node_list.findItems(item, Qt.MatchExactly)) == 0:
                self.node_list.addItem(item)
            items = self.node_list.findItems(item, Qt.MatchExactly)
            for item in items:
                item.setSelected(True)
        self.handle_node_item_changed()
