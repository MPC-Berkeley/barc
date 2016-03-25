#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import threading

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QStandardItem

from rqt_py_common.message_tree_model import MessageTreeModel
from rqt_py_common.data_items import ReadonlyItem, CheckableItem


class PublisherTreeModel(MessageTreeModel):
    _column_names = ['topic', 'type', 'rate', 'expression']
    item_value_changed = Signal(int, str, str, str, object)

    def __init__(self, parent=None):
        super(PublisherTreeModel, self).__init__(parent)
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)
        self.clear()

        self._item_change_lock = threading.Lock()
        self.itemChanged.connect(self.handle_item_changed)

    def clear(self):
        super(PublisherTreeModel, self).clear()
        self.setHorizontalHeaderLabels(self._column_names)

    def get_publisher_ids(self, index_list):
        return [item._user_data['publisher_id'] for item in self._get_toplevel_items(index_list)]

    def remove_items_with_parents(self, index_list):
        for item in self._get_toplevel_items(index_list):
            self.removeRow(item.row())

    def handle_item_changed(self, item):
        if not self._item_change_lock.acquire(False):
            #qDebug('PublisherTreeModel.handle_item_changed(): could not acquire lock')
            return
        # lock has been acquired
        topic_name = item._path
        column_name = self._column_names[item.column()]
        if item.isCheckable():
            new_value = str(item.checkState() == Qt.Checked)
        else:
            new_value = item.text().strip()
        #print 'PublisherTreeModel.handle_item_changed(): %s, %s, %s' % (topic_name, column_name, new_value)

        self.item_value_changed.emit(item._user_data['publisher_id'], topic_name, column_name, new_value, item.setText)

        # release lock
        self._item_change_lock.release()

    def remove_publisher(self, publisher_id):
        for top_level_row_number in range(self.rowCount()):
            item = self.item(top_level_row_number)
            if item is not None and item._user_data['publisher_id'] == publisher_id:
                self.removeRow(top_level_row_number)
                return top_level_row_number
        return None

    def update_publisher(self, publisher_info):
        top_level_row_number = self.remove_publisher(publisher_info['publisher_id'])
        self.add_publisher(publisher_info, top_level_row_number)

    def add_publisher(self, publisher_info, top_level_row_number=None):
        # recursively create widget items for the message's slots
        parent = self
        slot = publisher_info['message_instance']
        slot_name = publisher_info['topic_name']
        slot_type_name = publisher_info['message_instance']._type
        slot_path = publisher_info['topic_name']
        user_data = {'publisher_id': publisher_info['publisher_id']}
        kwargs = {
            'user_data': user_data,
            'top_level_row_number': top_level_row_number,
            'expressions': publisher_info['expressions'],
        }
        top_level_row = self._recursive_create_items(parent, slot, slot_name, slot_type_name, slot_path, **kwargs)

        # fill tree widget columns of top level item
        if publisher_info['enabled']:
            top_level_row[self._column_index['topic']].setCheckState(Qt.Checked)
        top_level_row[self._column_index['rate']].setText(str(publisher_info['rate']))

    def _get_data_items_for_path(self, slot_name, slot_type_name, slot_path, **kwargs):
        if slot_name.startswith('/'):
            return (CheckableItem(slot_name), ReadonlyItem(slot_type_name), QStandardItem(''), ReadonlyItem(''))
        expression_item = QStandardItem('')
        expression_item.setToolTip('enter valid Python expression here, using "i" as counter and functions from math, random and time modules')
        return (ReadonlyItem(slot_name), QStandardItem(slot_type_name), ReadonlyItem(''), expression_item)

    def _recursive_create_items(self, parent, slot, slot_name, slot_type_name, slot_path, expressions={}, **kwargs):
        row, is_leaf_node = super(PublisherTreeModel, self)._recursive_create_items(parent, slot, slot_name, slot_type_name, slot_path, expressions=expressions, **kwargs)
        if is_leaf_node:
            expression_text = expressions.get(slot_path, repr(slot))
            row[self._column_index['expression']].setText(expression_text)
        return row
