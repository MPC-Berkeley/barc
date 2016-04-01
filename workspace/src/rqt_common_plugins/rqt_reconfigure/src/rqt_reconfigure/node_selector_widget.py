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

from collections import OrderedDict
import os
import time

import dynamic_reconfigure as dyn_reconf
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import (QHeaderView, QItemSelectionModel,
                                     QWidget)
import rospy
from rospy.exceptions import ROSException
import rosservice

from rqt_py_common.rqt_ros_graph import RqtRosGraph
from rqt_reconfigure.filter_children_model import FilterChildrenModel
from rqt_reconfigure.treenode_qstditem import TreenodeQstdItem
from rqt_reconfigure.treenode_item_model import TreenodeItemModel

from rqt_reconfigure.dynreconf_client_widget import DynreconfClientWidget


class NodeSelectorWidget(QWidget):
    _COL_NAMES = ['Node']

    # public signal
    sig_node_selected = Signal(DynreconfClientWidget)

    def __init__(self, parent, rospack, signal_msg=None):
        """
        @param signal_msg: Signal to carries a system msg that is shown on GUI.
        @type signal_msg: QtCore.Signal
        """
        super(NodeSelectorWidget, self).__init__()
        self._parent = parent
        self.stretch = None
        self._signal_msg = signal_msg

        ui_file = os.path.join(rospack.get_path('rqt_reconfigure'), 'resource',
                               'node_selector.ui')
        loadUi(ui_file, self)

        # List of the available nodes. Since the list should be updated over
        # time and we don't want to create node instance per every update
        # cycle, This list instance should better be capable of keeping track.
        self._nodeitems = OrderedDict()
        # Dictionary. 1st elem is node's GRN name,
        # 2nd is TreenodeQstdItem instance.
        # TODO: Needs updated when nodes list updated.

        #  Setup treeview and models
        self._item_model = TreenodeItemModel()
        self._rootitem = self._item_model.invisibleRootItem()  # QStandardItem

        self._nodes_previous = None

        # Calling this method updates the list of the node.
        # Initially done only once.
        self._update_nodetree_pernode()

        # TODO(Isaac): Needs auto-update function enabled, once another
        #             function that updates node tree with maintaining
        #             collapse/expansion  state. http://goo.gl/GuwYp can be a
        #             help.

        self._collapse_button.pressed.connect(
                                          self._node_selector_view.collapseAll)
        self._expand_button.pressed.connect(self._node_selector_view.expandAll)
        self._refresh_button.pressed.connect(self._refresh_nodes)

        # Filtering preparation.
        self._proxy_model = FilterChildrenModel(self)
        self._proxy_model.setDynamicSortFilter(True)
        self._proxy_model.setSourceModel(self._item_model)
        self._node_selector_view.setModel(self._proxy_model)
        self._filterkey_prev = ''

        # This 1 line is needed to enable horizontal scrollbar. This setting
        # isn't available in .ui file.
        # Ref. http://stackoverflow.com/a/6648906/577001
        self._node_selector_view.header().setResizeMode(
                                              0, QHeaderView.ResizeToContents)

        # Setting slot for when user clicks on QTreeView.
        self.selectionModel = self._node_selector_view.selectionModel()
        # Note: self.selectionModel.currentChanged doesn't work to deselect
        # a treenode as expected. Need to use selectionChanged.
        self.selectionModel.selectionChanged.connect(
                                                  self._selection_changed_slot)

    def node_deselected(self, grn):
        """
        Deselect the index that corresponds to the given GRN.

        :type grn: str
        """

        # Obtain the corresponding index.
        qindex_tobe_deselected = self._item_model.get_index_from_grn(grn)
        rospy.logdebug('NodeSelWidt node_deselected qindex={} data={}'.format(
                                qindex_tobe_deselected,
                                qindex_tobe_deselected.data(Qt.DisplayRole)))

        # Obtain all indices currently selected.
        indexes_selected = self.selectionModel.selectedIndexes()
        for index in indexes_selected:
            grn_from_selectedindex = RqtRosGraph.get_upper_grn(index, '')
            rospy.logdebug(' Compare given grn={} grn from selected={}'.format(
                                                  grn, grn_from_selectedindex))
            # If GRN retrieved from selected index matches the given one.
            if grn == grn_from_selectedindex:
                # Deselect the index.
                self.selectionModel.select(index, QItemSelectionModel.Deselect)

    def node_selected(self, grn):
        """
        Select the index that corresponds to the given GRN.

        :type grn: str
        """

        # Obtain the corresponding index.
        qindex_tobe_selected = self._item_model.get_index_from_grn(grn)
        rospy.logdebug('NodeSelWidt node_selected qindex={} data={}'.format(
                                qindex_tobe_selected,
                                qindex_tobe_selected.data(Qt.DisplayRole)))


        # Select the index.
        if qindex_tobe_selected:
            self.selectionModel.select(qindex_tobe_selected, QItemSelectionModel.Select)

    def _selection_deselected(self, index_current, rosnode_name_selected):
        """
        Intended to be called from _selection_changed_slot.
        """
        self.selectionModel.select(index_current, QItemSelectionModel.Deselect)

        try:
            reconf_widget = self._nodeitems[
                                 rosnode_name_selected].get_dynreconf_widget()
        except ROSException as e:
            raise e

        # Signal to notify other pane that also contains node widget.
        self.sig_node_selected.emit(reconf_widget)
        #self.sig_node_selected.emit(self._nodeitems[rosnode_name_selected])

    def _selection_selected(self, index_current, rosnode_name_selected):
        """Intended to be called from _selection_changed_slot."""
        rospy.logdebug('_selection_changed_slot row={} col={} data={}'.format(
                          index_current.row(), index_current.column(),
                          index_current.data(Qt.DisplayRole)))

        # Determine if it's terminal treenode.
        found_node = False
        for nodeitem in self._nodeitems.itervalues():
            name_nodeitem = nodeitem.data(Qt.DisplayRole)
            name_rosnode_leaf = rosnode_name_selected[
                       rosnode_name_selected.rfind(RqtRosGraph.DELIM_GRN) + 1:]

            # If name of the leaf in the given name & the name taken from
            # nodeitem list matches.
            if ((name_nodeitem == rosnode_name_selected) and
                (name_nodeitem[name_nodeitem.rfind(RqtRosGraph.DELIM_GRN) + 1:]
                 == name_rosnode_leaf)):

                rospy.logdebug('terminal str {} MATCH {}'.format(
                                             name_nodeitem, name_rosnode_leaf))
                found_node = True
                break
        if not found_node:  # Only when it's NOT a terminal we deselect it.
            self.selectionModel.select(index_current,
                                       QItemSelectionModel.Deselect)
            return

        # Only when it's a terminal we move forward.

        item_child = self._nodeitems[rosnode_name_selected]
        item_widget = None
        try:
            item_widget = item_child.get_dynreconf_widget()
        except ROSException as e:
            raise e
        rospy.logdebug('item_selected={} child={} widget={}'.format(
                       index_current, item_child, item_widget))
        self.sig_node_selected.emit(item_widget)

        # Show the node as selected.
        #selmodel.select(index_current, QItemSelectionModel.SelectCurrent)

    def _selection_changed_slot(self, selected, deselected):
        """
        Sends "open ROS Node box" signal ONLY IF the selected treenode is the
        terminal treenode.
        Receives args from signal QItemSelectionModel.selectionChanged.

        :param selected: All indexs where selected (could be multiple)
        :type selected: QItemSelection
        :type deselected: QItemSelection
        """

        ## Getting the index where user just selected. Should be single.
        if not selected.indexes() and not deselected.indexes():
            rospy.logerr('Nothing selected? Not ideal to reach here')
            return

        index_current = None
        if selected.indexes():
            index_current = selected.indexes()[0]
        elif len(deselected.indexes()) == 1:
            # Setting length criteria as 1 is only a workaround, to avoid
            # Node boxes on right-hand side disappears when filter key doesn't
            # match them.
            # Indeed this workaround leaves another issue. Question for
            # permanent solution is asked here http://goo.gl/V4DT1
            index_current = deselected.indexes()[0]

        rospy.logdebug('  - - - index_current={}'.format(index_current))

        rosnode_name_selected = RqtRosGraph.get_upper_grn(index_current, '')

        # If retrieved node name isn't in the list of all nodes.
        if not rosnode_name_selected in self._nodeitems.keys():
            # De-select the selected item.
            self.selectionModel.select(index_current,
                                       QItemSelectionModel.Deselect)
            return

        if selected.indexes():
            try:
                self._selection_selected(index_current, rosnode_name_selected)
            except ROSException as e:
                #TODO: print to sysmsg pane
                err_msg = e.message + '. Connection to node=' + \
                          format(rosnode_name_selected) + ' failed'
                self._signal_msg.emit(err_msg)
                rospy.logerr(err_msg)

        elif deselected.indexes():
            try:
                self._selection_deselected(index_current,
                                           rosnode_name_selected)
            except ROSException as e:
                rospy.logerr(e.message)
                #TODO: print to sysmsg pane

    def get_paramitems(self):
        """
        :rtype: OrderedDict 1st elem is node's GRN name,
                2nd is TreenodeQstdItem instance
        """
        return self._nodeitems

    def _update_nodetree_pernode(self):
        """
        """

        # TODO(Isaac): 11/25/2012 dynamic_reconfigure only returns params that
        #             are associated with nodes. In order to handle independent
        #             params, different approach needs taken.
        try:
            nodes = dyn_reconf.find_reconfigure_services()
        except rosservice.ROSServiceIOException as e:
            rospy.logerr("Reconfigure GUI cannot connect to master.")
            raise e  # TODO Make sure 'raise' here returns or finalizes func.

        if not nodes == self._nodes_previous:
            i_node_curr = 1
            num_nodes = len(nodes)
            elapsedtime_overall = 0.0
            for node_name_grn in nodes:
                # Skip this grn if we already have it
                if node_name_grn in self._nodeitems:
                    i_node_curr += 1
                    continue

                time_siglenode_loop = time.time()

                ####(Begin) For DEBUG ONLY; skip some dynreconf creation
#                if i_node_curr % 2 != 0:
#                    i_node_curr += 1
#                    continue
                #### (End) For DEBUG ONLY. ####

                # Instantiate QStandardItem. Inside, dyn_reconf client will
                # be generated too.
                treenodeitem_toplevel = TreenodeQstdItem(
                                node_name_grn, TreenodeQstdItem.NODE_FULLPATH)
                _treenode_names = treenodeitem_toplevel.get_treenode_names()

                try:
                    treenodeitem_toplevel.connect_param_server()
                except rospy.exceptions.ROSException as e:
                    rospy.logerr(e.message)
                    #Skip item that fails to connect to its node.
                    continue
                    #TODO: Needs to show err msg on GUI too.

                # Using OrderedDict here is a workaround for StdItemModel
                # not returning corresponding item to index.
                self._nodeitems[node_name_grn] = treenodeitem_toplevel

                self._add_children_treenode(treenodeitem_toplevel,
                                            self._rootitem, _treenode_names)

                time_siglenode_loop = time.time() - time_siglenode_loop
                elapsedtime_overall += time_siglenode_loop

                _str_progress = 'reconf ' + \
                     'loading #{}/{} {} / {}sec node={}'.format(
                     i_node_curr, num_nodes, round(time_siglenode_loop, 2),
                     round(elapsedtime_overall, 2), node_name_grn)

                # NOT a debug print - please DO NOT remove. This print works
                # as progress notification when loading takes long time.
                rospy.logdebug(_str_progress)
                i_node_curr += 1

    def _add_children_treenode(self, treenodeitem_toplevel,
                               treenodeitem_parent, child_names_left):
        """
        Evaluate current treenode and the previous treenode at the same depth.
        If the name of both nodes is the same, current node instance is
        ignored (that means children will be added to the same parent). If not,
        the current node gets added to the same parent node. At the end, this
        function gets called recursively going 1 level deeper.

        :type treenodeitem_toplevel: TreenodeQstdItem
        :type treenodeitem_parent: TreenodeQstdItem.
        :type child_names_left: List of str
        :param child_names_left: List of strings that is sorted in hierarchical
                                 order of params.
        """
        # TODO(Isaac): Consider moving this method to rqt_py_common.

        name_currentnode = child_names_left.pop(0)
        grn_curr = treenodeitem_toplevel.get_raw_param_name()
        stditem_currentnode = TreenodeQstdItem(grn_curr,
                                               TreenodeQstdItem.NODE_FULLPATH)

        # item at the bottom is your most recent node.
        row_index_parent = treenodeitem_parent.rowCount() - 1

        # Obtain and instantiate prev node in the same depth.
        name_prev = ''
        stditem_prev = None
        if treenodeitem_parent.child(row_index_parent):
            stditem_prev = treenodeitem_parent.child(row_index_parent)
            name_prev = stditem_prev.text()

        stditem = None
        # If the name of both nodes is the same, current node instance is
        # ignored (that means children will be added to the same parent)
        if name_prev != name_currentnode:
            stditem_currentnode.setText(name_currentnode)

            # Arrange alphabetically by display name
            insert_index = 0
            while insert_index < treenodeitem_parent.rowCount() and treenodeitem_parent.child(insert_index).text() < name_currentnode:
                insert_index += 1

            treenodeitem_parent.insertRow(insert_index, stditem_currentnode)
            stditem = stditem_currentnode
        else:
            stditem = stditem_prev

        if child_names_left:
            # TODO: Model is closely bound to a certain type of view (treeview)
            # here. Ideally isolate those two. Maybe we should split into 2
            # class, 1 handles view, the other does model.
            self._add_children_treenode(treenodeitem_toplevel, stditem,
                                        child_names_left)
        else:  # Selectable ROS Node.
            #TODO: Accept even non-terminal treenode as long as it's ROS Node.
            self._item_model.set_item_from_index(grn_curr, stditem.index())

    def _prune_nodetree_pernode(self):
        try:
            nodes = dyn_reconf.find_reconfigure_services()
        except rosservice.ROSServiceIOException as e:
            rospy.logerr("Reconfigure GUI cannot connect to master.")
            raise e  # TODO Make sure 'raise' here returns or finalizes func.

        for i in reversed(range(0, self._rootitem.rowCount())):
            candidate_for_removal = self._rootitem.child(i).get_raw_param_name()
            if not candidate_for_removal in nodes:
                rospy.logdebug('Removing {} because the server is no longer available.'.format(
                                   candidate_for_removal))
                self._nodeitems[candidate_for_removal].disconnect_param_server()
                self._rootitem.removeRow(i)
                self._nodeitems.pop(candidate_for_removal)

    def _refresh_nodes(self):
        self._prune_nodetree_pernode()
        self._update_nodetree_pernode()

    def close_node(self):
        rospy.logdebug(" in close_node")
        # TODO(Isaac) Figure out if dynamic_reconfigure needs to be closed.

    def set_filter(self, filter_):
        """
        Pass fileter instance to the child proxymodel.
        :type filter_: BaseFilter
        """
        self._proxy_model.set_filter(filter_)

    def _test_sel_index(self, selected, deselected):
        """
        Method for Debug only
        """
        #index_current = self.selectionModel.currentIndex()
        src_model = self._item_model
        index_current = None
        index_deselected = None
        index_parent = None
        curr_qstd_item = None
        if selected.indexes():
            index_current = selected.indexes()[0]
            index_parent = index_current.parent()
            curr_qstd_item = src_model.itemFromIndex(index_current)
        elif deselected.indexes():
            index_deselected = deselected.indexes()[0]
            index_parent = index_deselected.parent()
            curr_qstd_item = src_model.itemFromIndex(index_deselected)

        if selected.indexes() > 0:
            rospy.logdebug('sel={} par={} desel={} sel.d={} par.d={}'.format(
                                 index_current, index_parent, index_deselected,
                                 index_current.data(Qt.DisplayRole),
                                 index_parent.data(Qt.DisplayRole),)
                                 + ' desel.d={} cur.item={}'.format(
                                 None,  # index_deselected.data(Qt.DisplayRole)
                                 curr_qstd_item))
        elif deselected.indexes():
            rospy.logdebug('sel={} par={} desel={} sel.d={} par.d={}'.format(
                                 index_current, index_parent, index_deselected,
                                 None, index_parent.data(Qt.DisplayRole)) +
                           ' desel.d={} cur.item={}'.format(
                                 index_deselected.data(Qt.DisplayRole),
                                 curr_qstd_item))
