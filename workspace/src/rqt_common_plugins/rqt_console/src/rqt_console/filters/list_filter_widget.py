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


class ListFilterWidget(QWidget):
    """
    Generic List widget to be used when implementing filters that require
    limited dynamic selections
    """
    def __init__(self, parentfilter, rospack, item_provider):
        """
        :param parentfilter: The filter object, must implement set_list and
        contain _list ''QObject''
        :param item_provider: a function designed to provide a list or dict
        """
        super(ListFilterWidget, self).__init__()
        ui_file = os.path.join(rospack.get_path('rqt_console'), 'resource/filters', 'list_filter_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('ListFilterWidget')
        self._parentfilter = parentfilter  # When data is changed we need to store it in the parent filter

        # keep color for highlighted items even when not active
        active_color = self.palette().brush(QPalette.Active, QPalette.Highlight).color().name()
        self.setStyleSheet('QListWidget:item:selected:!active { background: %s; }' % active_color)

        self._list_populate_function = item_provider
        self.list_widget.itemSelectionChanged.connect(self.handle_item_changed)
        self._display_list = []
        self.repopulate()

    def select_item(self, text):
        """
        All items matching text will be selected in the list_widget
        :param item: a string to be matched against the list ''str''
        """
        items = self.list_widget.findItems(text, Qt.MatchExactly)
        for item in items:
            item.setSelected(True)
        self.handle_item_changed()

    def handle_item_changed(self):
        self._parentfilter.set_selected_items(self.list_widget.selectedItems())

    def repopulate(self):
        """
        Repopulates the display widgets based on the function arguments passed
        in during initialization
        """
        new_items = self._list_populate_function()

        new_set = set(new_items.values() if isinstance(new_items, dict) else new_items)

        if len(new_items) != len(self._display_list):
            if isinstance(new_items, dict):
                # for dictionaries store the key as user role data
                for key in sorted(new_items.keys()):
                    item = new_items[key]
                    if item not in self._display_list:
                        self.list_widget.addItem(item)
                        self.list_widget.item(self.list_widget.count() - 1).setData(Qt.UserRole, key)
            else:
                for item in new_items:
                    if item not in self._display_list:
                        self._add_item(item)
        self._display_list = sorted(set(new_set) | set(self._display_list))

    def _add_item(self, item):
        """
        Insert item in alphabetical order.
        """
        for i in range(self.list_widget.count()):
            if item <= self.list_widget.item(i).text():
                self.list_widget.insertItem(i, item)
                return
        self.list_widget.addItem(item)

    def save_settings(self, settings):
        """
        Saves the settings for this filter.
        :param settings: used to write the settings to an ini file ''qt_gui.settings.Settings''
        """
        settings.set_value('itemlist', pack(self._parentfilter._selected_items))

    def restore_settings(self, settings):
        """
        Restores the settings for this filter from an ini file.
        :param settings: used to extract the settings from an ini file ''qt_gui.settings.Settings''
        """
        for index in range(self.list_widget.count()):
            self.list_widget.item(index).setSelected(False)
        item_list = unpack(settings.value('itemlist'))
        for item in item_list:
            if len(self.list_widget.findItems(item, Qt.MatchExactly)) == 0:
                self.list_widget.addItem(item)
            self.select_item(item)
