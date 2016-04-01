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

import rospy
from python_qt_binding.QtGui import QStandardItem, QStandardItemModel
from .data_items import ReadonlyItem


class MessageTreeModel(QStandardItemModel):

    def __init__(self, parent=None):
        # FIXME: why is this not working? should be the same as the following line...
        #super(MessageTreeModel, self).__init__(parent)
        QStandardItemModel.__init__(self, parent)

    def add_message(self, message_instance, message_name='', message_type='', message_path=''):
        if message_instance is None:
            return
        self._recursive_create_items(self, message_instance, message_name, message_type, message_path)

    def _get_toplevel_items(self, index_list):
        items = [self.itemFromIndex(index) for index in index_list]
        uniqueItems = {}
        for item in items:
            while item.parent() is not None:
                item = item.parent()
            if item.row() not in uniqueItems:
                uniqueItems[item.row()] = item
        return uniqueItems.values()

    def _get_data_items_for_path(self, slot_name, slot_type_name, slot_path, **kwargs):
        return (QStandardItem(slot_name), QStandardItem(slot_type_name), QStandardItem(slot_path))

    def _recursive_create_items(self, parent, slot, slot_name, slot_type_name, slot_path, **kwargs):
        row = []
        for item in self._get_data_items_for_path(slot_name, slot_type_name, slot_path, **kwargs):
            item._path = slot_path
            item._user_data = kwargs.get('user_data', None)
            row.append(item)

        is_leaf_node = False
        if hasattr(slot, '__slots__') and hasattr(slot, '_slot_types'):
            for child_slot_name, child_slot_type in zip(slot.__slots__, slot._slot_types):
                child_slot_path = slot_path + '/' + child_slot_name
                child_slot = getattr(slot, child_slot_name)
                self._recursive_create_items(row[0], child_slot, child_slot_name, child_slot_type, child_slot_path, **kwargs)

        elif type(slot) in (list, tuple) and (len(slot) > 0):
            child_slot_type = slot_type_name[:slot_type_name.find('[')]
            for index, child_slot in enumerate(slot):
                child_slot_name = '[%d]' % index
                child_slot_path = slot_path + child_slot_name
                self._recursive_create_items(row[0], child_slot, child_slot_name, child_slot_type, child_slot_path, **kwargs)

        else:
            is_leaf_node = True

        if parent is self and kwargs.get('top_level_row_number', None) is not None:
            parent.insertRow(kwargs['top_level_row_number'], row)
        else:
            parent.appendRow(row)

        return (row, is_leaf_node)

    '''
    NOTE: I (Isaac Saito) suspect that this function might have same/similar
          functionality with _recursive_create_items.

    @summary: Evaluate current node and the previous node on the same depth.
              If the name of both nodes at the same depth is the same,
              current name is added to the previous node.
              If not, the current name gets added to the parent node.
              At the end, this function calls itself recursively going down
              1 level deeper.
    @param stditem_parent: QStandardItem.
    @param names_on_branch: List of strings each of which
                            represents the name of the node.
                            Ex. If you have a hierarchy that looks like:

                                 /top_node/sub_node/subsub_node

                            then this list would look like:

                                 [ top_node, sub_node, subsub_node ]
    @author: Isaac Saito
    '''
    @staticmethod
    def _build_tree_recursive(stditem_parent, names_on_branch):
        name_curr = names_on_branch.pop(0)
        stditem_curr = ReadonlyItem(name_curr)

        row_index_parent = stditem_parent.rowCount() - 1
        # item at the bottom is your most recent node.

        name_prev = ''
        stditem_prev = None
        if not stditem_parent.child(row_index_parent) == None:
            stditem_prev = stditem_parent.child(row_index_parent)
            name_prev = stditem_prev.text()

        stditem = None
        if not name_prev == name_curr:
            stditem_parent.appendRow(stditem_curr)
            stditem = stditem_curr
        else:
            stditem = stditem_prev

        rospy.logdebug('add_tree_node 1 name_curr=%s ' + '\n\t\t\t\t\tname_prev=%s row_index_parent=%d', name_curr, name_prev, row_index_parent)
        if (0 < len(names_on_branch)):
            MessageTreeModel._build_tree_recursive(stditem, names_on_branch)
