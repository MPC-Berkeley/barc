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

from __future__ import division

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QSortFilterProxyModel
import rospy

from rqt_reconfigure.treenode_qstditem import TreenodeQstdItem


class FilterChildrenModel(QSortFilterProxyModel):
    """
    Extending QSortFilterProxyModel, this provides methods to filter children
    tree nodes.

    QSortFilterProxyModel filters top-down direction starting from the
    top-level of tree, and once a node doesn't hit the query it gets disabled.
    Filtering with this class reflects the result from the bottom node.

    Ex.
    #TODO example needed here
    """

    # Emitted when parameters filtered. int indicates the order/index of
    # params displayed.
    sig_filtered = Signal(int)

    def __init__(self, parent):
        super(FilterChildrenModel, self).__init__(parent)

        # :Key: Internal ID of QModelIndex of each treenode.
        # :Value: TreenodeStatus
        # self._treenodes = OrderedDict()

        self._parent = parent
        self._toplv_parent_prev = None

    def filterAcceptsRow(self, src_row, src_parent_qmindex):
        """
        Overridden.

        Terminology:
        "Treenode" is deliberately used to avoid confusion with "Node" in ROS.

        :type src_row: int
        :type src_parent_qmindex: QModelIndex
        """
        rospy.logdebug('filerAcceptRow 1')
        return self._filter_row_recur(src_row, src_parent_qmindex)

    def _filter_row_recur(self, src_row, src_parent_qmindex):
        """
        :type src_row: int
        :type src_parent_qmindex: QModelIndex
        """
        _src_model = self.sourceModel()
        curr_qmindex = _src_model.index(src_row, 0, src_parent_qmindex)
        curr_qitem = _src_model.itemFromIndex(curr_qmindex)

        if isinstance(curr_qitem, TreenodeQstdItem):
            # If selectable ROS Node, get GRN name
            nodename_fullpath = curr_qitem.get_raw_param_name()
            text_filter_target = nodename_fullpath
            rospy.logdebug('   Nodename full={} '.format(nodename_fullpath))
        else:
            # If ReadonlyItem, this means items are the parameters, not a part
            # of node name. So, get param name.
            text_filter_target = curr_qitem.data(Qt.DisplayRole)

        regex = self.filterRegExp()
        pos_hit = regex.indexIn(text_filter_target)
        if pos_hit >= 0:  # Query hit.
            rospy.logdebug('curr data={} row={} col={}'.format(
                                                        curr_qmindex.data(),
                                                        curr_qmindex.row(),
                                                        curr_qmindex.column()))

            # Set all subsequent treenodes True
            rospy.logdebug(' FCModel.filterAcceptsRow src_row={}'.format(
                            src_row) +
                           ' parent row={} data={}'.format(
                              src_parent_qmindex.row(),
                              src_parent_qmindex.data()) +
                           ' filterRegExp={}'.format(regex))

            # If the index is the terminal treenode, parameters that hit
            # the query are displayed at the root tree.
            _child_index = curr_qmindex.child(0, 0)
            if ((not _child_index.isValid()) and
                (isinstance(curr_qitem, TreenodeQstdItem))):
                self._show_params_view(src_row, curr_qitem)

            # Once we find a treenode that hits the query, no need to further
            # traverse since what this method wants to know with the given
            # index is whether the given index is supposed to be shown or not.
            # Thus, just return True here.
            return True

        if not isinstance(curr_qitem, TreenodeQstdItem):
            return False  # If parameters, no need for recursive filtering.

        # Evaluate children recursively.
        row_child = 0
        while True:
            child_qmindex = curr_qmindex.child(row_child, 0)
            if child_qmindex.isValid():
                flag = self._filter_row_recur(row_child, curr_qmindex)
                if flag:
                    return True
            else:
                return False
            row_child += 1
        return False

    def _show_params_view(self, src_row, curr_qitem):
        """
        :type curr_qitem: QStandardItem
        """

        rospy.logdebug('_show_params_view data={}'.format(
                                  curr_qitem.data(Qt.DisplayRole)))
        curr_qitem.enable_param_items()

    def _get_toplevel_parent_recur(self, qmindex):
        p = qmindex.parent()
        if p.isValid():
            self._get_toplevel_parent(p)
        return p

    def filterAcceptsColumn(self, source_column, source_parent):
        """
        Overridden.

        Doing nothing really since columns are not in use.

        :type source_column: int
        :type source_parent: QModelIndex
        """
        rospy.logdebug('FCModel.filterAcceptsCol source_col={} '.format(
            source_column) + 'parent col={} row={} data={}'.format(
            source_parent.column(), source_parent.row(), source_parent.data()))
        return True

    def set_filter(self, filter_):
        self._filter = filter_

        # If filtered text is '' (0-length str), invalidate current
        # filtering, in the hope of making filtering process faster.
        if filter_.get_text == '':
            self.invalidate()
            rospy.loginfo('filter invalidated.')

        # By calling setFilterRegExp, filterAccepts* methods get kicked.
        self.setFilterRegExp(self._filter.get_regexp())
