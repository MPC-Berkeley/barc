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

import rospkg
import sys

from python_qt_binding.QtCore import Signal, QMargins
from python_qt_binding.QtGui import (QLabel, QHBoxLayout, QSplitter,
                                     QVBoxLayout, QWidget, QItemSelectionModel)

from rqt_reconfigure.node_selector_widget import NodeSelectorWidget
from rqt_reconfigure.paramedit_widget import ParameditWidget
from rqt_reconfigure.text_filter import TextFilter
from rqt_reconfigure.text_filter_widget import TextFilterWidget
import rospy

class ParamWidget(QWidget):
    _TITLE_PLUGIN = 'Dynamic Reconfigure'

    # To be connected to PluginContainerWidget
    sig_sysmsg = Signal(str)
    sig_sysprogress = Signal(str)

    # To make selections from CLA
    sig_selected = Signal(str)

    def __init__(self, context, node=None):
        """
        This class is intended to be called by rqt plugin framework class.
        Currently (12/12/2012) the whole widget is splitted into 2 panes:
        one on left allows you to choose the node(s) you work on. Right side
        pane lets you work with the parameters associated with the node(s) you
        select on the left.

        (12/27/2012) Despite the pkg name is changed to rqt_reconfigure to
        reflect the available functionality, file & class names remain
        'param', expecting all the parameters will become handle-able.
        """

        super(ParamWidget, self).__init__()
        self.setObjectName(self._TITLE_PLUGIN)
        self.setWindowTitle(self._TITLE_PLUGIN)

        rp = rospkg.RosPack()

        #TODO: .ui file needs to replace the GUI components declaration
        #            below. For unknown reason, referring to another .ui files
        #            from a .ui that is used in this class failed. So for now,
        #            I decided not use .ui in this class.
        #            If someone can tackle this I'd appreciate.
        _hlayout_top = QHBoxLayout(self)
        _hlayout_top.setContentsMargins(QMargins(0, 0, 0, 0))
        self._splitter = QSplitter(self)
        _hlayout_top.addWidget(self._splitter)

        _vlayout_nodesel_widget = QWidget()
        _vlayout_nodesel_side = QVBoxLayout()
        _hlayout_filter_widget = QWidget(self)
        _hlayout_filter = QHBoxLayout()
        self._text_filter = TextFilter()
        self.filter_lineedit = TextFilterWidget(self._text_filter, rp)
        self.filterkey_label = QLabel("&Filter key:")
        self.filterkey_label.setBuddy(self.filter_lineedit)
        _hlayout_filter.addWidget(self.filterkey_label)
        _hlayout_filter.addWidget(self.filter_lineedit)
        _hlayout_filter_widget.setLayout(_hlayout_filter)
        self._nodesel_widget = NodeSelectorWidget(self, rp, self.sig_sysmsg)
        _vlayout_nodesel_side.addWidget(_hlayout_filter_widget)
        _vlayout_nodesel_side.addWidget(self._nodesel_widget)
        _vlayout_nodesel_side.setSpacing(1)
        _vlayout_nodesel_widget.setLayout(_vlayout_nodesel_side)

        reconf_widget = ParameditWidget(rp)

        self._splitter.insertWidget(0, _vlayout_nodesel_widget)
        self._splitter.insertWidget(1, reconf_widget)
        # 1st column, _vlayout_nodesel_widget, to minimize width.
        # 2nd col to keep the possible max width.
        self._splitter.setStretchFactor(0, 0)
        self._splitter.setStretchFactor(1, 1)

        # Signal from paramedit widget to node selector widget.
        reconf_widget.sig_node_disabled_selected.connect(
                                       self._nodesel_widget.node_deselected)
        # Pass name of node to editor widget
        self._nodesel_widget.sig_node_selected.connect(
                                                     reconf_widget.show_reconf)

        if not node:
            title = self._TITLE_PLUGIN
        else:
            title = self._TITLE_PLUGIN + ' %s' % node
        self.setObjectName(title)

        #Connect filter signal-slots.
        self._text_filter.filter_changed_signal.connect(
                                            self._filter_key_changed)

        # Open any clients indicated from command line
        self.sig_selected.connect(self._nodesel_widget.node_selected)
        for rn in [rospy.resolve_name(c) for c in context.argv()]:
            if rn in self._nodesel_widget.get_paramitems():
                self.sig_selected.emit(rn)
            else:
                rospy.logwarn('Could not find a dynamic reconfigure client named \'%s\'', str(rn))

    def shutdown(self):
        #TODO: Needs implemented. Trigger dynamic_reconfigure to unlatch
        #            subscriber.
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('splitter', self._splitter.saveState())

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('splitter'):
            self._splitter.restoreState(instance_settings.value('splitter'))
        else:
            self._splitter.setSizes([100, 100, 200])

    def get_filter_text(self):
        """
        :rtype: QString
        """
        return self.filter_lineedit.text()

    def _filter_key_changed(self):
        self._nodesel_widget.set_filter(self._text_filter)

    #TODO: This method should be integrated into common architecture. I just
    # can't think of how to do so within current design.
    def emit_sysmsg(self, msg_str):
        self.sig_sysmsg.emit(msg_str)


if __name__ == '__main__':
    # main should be used only for debug purpose.
    # This launches this QWidget as a standalone rqt gui.
    from rqt_gui.main import Main

    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_reconfigure'))
