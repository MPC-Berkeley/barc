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

from python_qt_binding.QtCore import Slot, QMimeData, QModelIndex, Qt, qWarning
from python_qt_binding.QtGui import QAction, QDrag, QHeaderView, QIcon, QMenu, QTreeView


class MessageTreeWidget(QTreeView):

    def __init__(self, parent=None):
        super(MessageTreeWidget, self).__init__(parent)
        self.setDragEnabled(True)
        self.sortByColumn(0, Qt.AscendingOrder)

        self.header().setResizeMode(QHeaderView.ResizeToContents)
        self.header().setContextMenuPolicy(Qt.CustomContextMenu)
        self.header().customContextMenuRequested.connect(self.handle_header_view_customContextMenuRequested)

        self._action_item_expand = QAction(QIcon.fromTheme('zoom-in'), 'Expand Selected', self)
        self._action_item_expand.triggered.connect(self._handle_action_item_expand)
        self._action_item_collapse = QAction(QIcon.fromTheme('zoom-out'), 'Collapse Selected', self)
        self._action_item_collapse.triggered.connect(self._handle_action_item_collapse)
        self.customContextMenuRequested.connect(self.handle_customContextMenuRequested)

    def startDrag(self, supportedActions):
        index = self.currentIndex()
        if not index.isValid():
            return

        item = self.model().itemFromIndex(index)
        path = getattr(item, '_path', None)
        if path is None:
            qWarning('MessageTreeWidget.startDrag(): no _path set on item %s' % item)
            return

        data = QMimeData()
        data.setText(item._path)

        drag = QDrag(self)
        drag.setMimeData(data)
        drag.exec_()

    @Slot('QPoint')
    def handle_customContextMenuRequested(self, pos):
        # show context menu
        menu = QMenu(self)
        self._context_menu_add_actions(menu, pos)
        menu.exec_(self.mapToGlobal(pos))

    def _context_menu_add_actions(self, menu, pos):
        if self.selectionModel().hasSelection():
            menu.addAction(self._action_item_expand)
            menu.addAction(self._action_item_collapse)

    def _handle_action_item_collapse(self):
        self._handle_action_set_expanded(False)

    def _handle_action_item_expand(self):
        self._handle_action_set_expanded(True)

    def _handle_action_set_expanded(self, expanded):
        def recursive_set_expanded(index):
            if index != QModelIndex():
                self.setExpanded(index, expanded)
                recursive_set_expanded(index.child(0, 0))
        for index in self.selectedIndexes():
            recursive_set_expanded(index)

    @Slot('QPoint')
    def handle_header_view_customContextMenuRequested(self, pos):

        # create context menu
        menu = QMenu(self)

        action_toggle_auto_resize = menu.addAction('Auto-Resize')
        action_toggle_auto_resize.setCheckable(True)
        auto_resize_flag = (self.header().resizeMode(0) == QHeaderView.ResizeToContents)
        action_toggle_auto_resize.setChecked(auto_resize_flag)

        action_toggle_sorting = menu.addAction('Sorting')
        action_toggle_sorting.setCheckable(True)
        action_toggle_sorting.setChecked(self.isSortingEnabled())

        # show menu
        action = menu.exec_(self.header().mapToGlobal(pos))

        # evaluate user action
        if action is action_toggle_auto_resize:
            if auto_resize_flag:
                self.header().setResizeMode(QHeaderView.Interactive)
            else:
                self.header().setResizeMode(QHeaderView.ResizeToContents)

        elif action is action_toggle_sorting:
            self.setSortingEnabled(not self.isSortingEnabled())
