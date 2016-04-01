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

from python_qt_binding.QtCore import Qt, qWarning
from python_qt_binding.QtGui import QBrush, QColor, QSortFilterProxyModel

from .filters.filter_collection import FilterCollection
from .message import Message


class MessageProxyModel(QSortFilterProxyModel):
    """
    Provides sorting and filtering capabilities for the MessageDataModel.
    Filtering is based on a collection of exclude and highlight filters.
    """

    def __init__(self):
        super(MessageProxyModel, self).__init__()
        self.setDynamicSortFilter(True)
        self.setFilterRole(Qt.UserRole)
        self.setSortCaseSensitivity(Qt.CaseInsensitive)
        self.setSortRole(Qt.UserRole)

        self._exclude_filters = FilterCollection()
        self._highlight_filters = FilterCollection()
        self._show_highlighted_only = False

        # caching source model locally for better performance of filterAcceptsRow()
        self._source_model = None

    def setSourceModel(self, source_model):
        super(MessageProxyModel, self).setSourceModel(source_model)
        self._source_model = self.sourceModel()

    # BEGIN Required implementations of QSortFilterProxyModel functions

    def filterAcceptsRow(self, sourcerow, sourceparent):
        """
        returns: True if the row does not match any exclude filter AND (_show_highlighted_only is False OR it matches any highlight filter), ''bool''
        """
        msg = self._source_model._messages[sourcerow]
        if self._exclude_filters.test_message(msg):
            # hide excluded message
            return False

        highlighted = True
        if self._highlight_filters.count_enabled_filters() > 0:
            highlighted = self._highlight_filters.test_message(msg, default=True)
        if self._show_highlighted_only and not highlighted:
            # hide messages which are not highlighted when only highlightes messages should be visible
            return False

        # update message state
        msg.highlighted = highlighted

        return True

    def data(self, proxy_index, role=None):
        """
        Set colors of items based on highlight filters.
        """
        index = self.mapToSource(proxy_index)
        if role == Qt.ForegroundRole:
            msg = self._source_model._messages[index.row()]
            if not msg.highlighted:
                return QBrush(Qt.gray)
        return self._source_model.data(index, role)

    # END Required implementations of QSortFilterProxyModel functions

    def handle_exclude_filters_changed(self):
        """
        Invalidate filters and trigger refiltering.
        """
        self.invalidateFilter()

    def handle_highlight_filters_changed(self):
        """
        Invalidate filters and trigger refiltering.
        """
        if self._show_highlighted_only:
            self.invalidateFilter()
        else:
            self.invalidateFilter()
            self.dataChanged.emit(self.index(0, 0), self.index(self.rowCount() - 1, self.columnCount() - 1))

    def add_exclude_filter(self, newfilter):
        self._exclude_filters.append(newfilter)

    def add_highlight_filter(self, newfilter):
        self._highlight_filters.append(newfilter)

    def delete_exclude_filter(self, index):
        del self._exclude_filters[index]
        self.handle_exclude_filters_changed()

    def delete_highlight_filter(self, index):
        del self._highlight_filters[index]
        self.handle_highlight_filters_changed()

    def set_show_highlighted_only(self, show_highlighted_only):
        self._show_highlighted_only = show_highlighted_only
        self.invalidateFilter()
