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
# Author: Isaac Saito, Ze'ev Klapow

import time

from python_qt_binding.QtCore import QSize, Qt, Signal, QMargins
from python_qt_binding.QtGui import (QFont, QFormLayout, QHBoxLayout, QIcon,
                                     QGroupBox, QLabel, QPushButton,
                                     QTabWidget, QVBoxLayout, QWidget)
import rospy

# *Editor classes that are not explicitly used within this .py file still need
# to be imported. They are invoked implicitly during runtime.
from .param_editors import BooleanEditor, DoubleEditor, EditorWidget, \
                           EDITOR_TYPES, EnumEditor, IntegerEditor, \
                           StringEditor

_GROUP_TYPES = {
    '': 'BoxGroup',
    'collapse': 'CollapseGroup',
    'tab': 'TabGroup',
    'hide': 'HideGroup',
    'apply': 'ApplyGroup',
}


def find_cfg(config, name):
    '''
    (Ze'ev) reaaaaallly cryptic function which returns the config object for
    specified group.
    '''
    cfg = None
    for k, v in config.items():
        try:
            if k.lower() == name.lower():
                cfg = v
                return cfg
            else:
                try:
                    cfg = find_cfg(v, name)
                    if cfg:
                        return cfg
                except Exception as exc:
                    raise exc
        except AttributeError:
            pass
        except Exception as exc:
            raise exc
    return cfg


class GroupWidget(QWidget):
    '''
    (Isaac's guess as of 12/13/2012)
    This class bonds multiple Editor instances that are associated with
    a single node as a group.
    '''

    # public signal
    sig_node_disabled_selected = Signal(str)
    sig_node_state_change = Signal(bool)

    def __init__(self, updater, config, nodename):
        '''
        :param config:
        :type config: Dictionary? defined in dynamic_reconfigure.client.Client
        :type nodename: str
        '''

        super(GroupWidget, self).__init__()
        self.state = config['state']
        self.param_name = config['name']
        self._toplevel_treenode_name = nodename

        # TODO: .ui file needs to be back into usage in later phase.
#        ui_file = os.path.join(rp.get_path('rqt_reconfigure'),
#                               'resource', 'singlenode_parameditor.ui')
#        loadUi(ui_file, self)

        verticalLayout = QVBoxLayout(self)
        verticalLayout.setContentsMargins(QMargins(0, 0, 0, 0))

        _widget_nodeheader = QWidget()
        _h_layout_nodeheader = QHBoxLayout(_widget_nodeheader)
        _h_layout_nodeheader.setContentsMargins(QMargins(0, 0, 0, 0))

        self.nodename_qlabel = QLabel(self)
        font = QFont('Trebuchet MS, Bold')
        font.setUnderline(True)
        font.setBold(True)

        # Button to close a node.
        _icon_disable_node = QIcon.fromTheme('window-close')
        _bt_disable_node = QPushButton(_icon_disable_node, '', self)
        _bt_disable_node.setToolTip('Hide this node')
        _bt_disable_node_size = QSize(36, 24)
        _bt_disable_node.setFixedSize(_bt_disable_node_size)
        _bt_disable_node.pressed.connect(self._node_disable_bt_clicked)

        _h_layout_nodeheader.addWidget(self.nodename_qlabel)
        _h_layout_nodeheader.addWidget(_bt_disable_node)

        self.nodename_qlabel.setAlignment(Qt.AlignCenter)
        font.setPointSize(10)
        self.nodename_qlabel.setFont(font)
        grid_widget = QWidget(self)
        self.grid = QFormLayout(grid_widget)
        verticalLayout.addWidget(_widget_nodeheader)
        verticalLayout.addWidget(grid_widget, 1)
        # Again, these UI operation above needs to happen in .ui file.

        self.tab_bar = None  # Every group can have one tab bar
        self.tab_bar_shown = False

        self.updater = updater

        self.editor_widgets = []
        self._param_names = []

        self._create_node_widgets(config)

        rospy.logdebug('Groups node name={}'.format(nodename))
        self.nodename_qlabel.setText(nodename)

        # Labels should not stretch
        #self.grid.setColumnStretch(1, 1)
        #self.setLayout(self.grid)

    def collect_paramnames(self, config):
        pass

    def _create_node_widgets(self, config):
        '''
        :type config: Dict?
        '''
        i_debug = 0
        for param in config['parameters']:
            begin = time.time() * 1000
            editor_type = '(none)'

            if param['edit_method']:
                widget = EnumEditor(self.updater, param)
            elif param['type'] in EDITOR_TYPES:
                rospy.logdebug('GroupWidget i_debug=%d param type =%s',
                               i_debug,
                               param['type'])
                editor_type = EDITOR_TYPES[param['type']]
                widget = eval(editor_type)(self.updater, param)

            self.editor_widgets.append(widget)
            self._param_names.append(param['name'])

            rospy.logdebug('groups._create_node_widgets num editors=%d',
                           i_debug)

            end = time.time() * 1000
            time_elap = end - begin
            rospy.logdebug('ParamG editor={} loop=#{} Time={}msec'.format(
                                              editor_type, i_debug, time_elap))
            i_debug += 1

        for name, group in config['groups'].items():
            if group['type'] == 'tab':
                widget = TabGroup(self, self.updater, group, self._toplevel_treenode_name)
            elif group['type'] in _GROUP_TYPES.keys():
                widget = eval(_GROUP_TYPES[group['type']])(self.updater, group, self._toplevel_treenode_name)
            else:
                widget = eval(_GROUP_TYPES[''])(self.updater, group, self._toplevel_treenode_name)

            self.editor_widgets.append(widget)
            rospy.logdebug('groups._create_node_widgets ' +
                           'name=%s',
                           name)

        for i, ed in enumerate(self.editor_widgets):
            ed.display(self.grid)

        rospy.logdebug('GroupWdgt._create_node_widgets len(editor_widgets)=%d',
                       len(self.editor_widgets))

    def display(self, grid):
        grid.addRow(self)

    def update_group(self, config):
        if 'state' in config:
            old_state = self.state
            self.state = config['state']
            if self.state != old_state:
                self.sig_node_state_change.emit(self.state)

        names = [name for name in config.keys()]

        for widget in self.editor_widgets:
            if isinstance(widget, EditorWidget):
                if widget.param_name in names:
                    widget.update_value(config[widget.param_name])
            elif isinstance(widget, GroupWidget):
                cfg = find_cfg(config, widget.param_name)
                widget.update_group(cfg)

    def close(self):
        for w in self.editor_widgets:
            w.close()

    def get_treenode_names(self):
        '''
        :rtype: str[]
        '''
        return self._param_names

    def _node_disable_bt_clicked(self):
        rospy.logdebug('param_gs _node_disable_bt_clicked')
        self.sig_node_disabled_selected.emit(self._toplevel_treenode_name)


class BoxGroup(GroupWidget):
    def __init__(self, updater, config, nodename):
        super(BoxGroup, self).__init__(updater, config, nodename)

        self.box = QGroupBox(self.param_name)
        self.box.setLayout(self.grid)

    def display(self, grid):
        grid.addRow(self.box)


class CollapseGroup(BoxGroup):
    def __init__(self, updater, config, nodename):
        super(CollapseGroup, self).__init__(updater, config, nodename)
        self.box.setCheckable(True)
        self.box.clicked.connect(self.click_cb)
        self.sig_node_state_change.connect(self.box.setChecked)

        for child in self.box.children():
            if child.isWidgetType():
                self.box.toggled.connect(child.setShown)

        self.box.setChecked(self.state)

    def click_cb(self, on):
        self.updater.update({'groups': {self.param_name: on}})


class HideGroup(BoxGroup):
    def __init__(self, updater, config, nodename):
        super(HideGroup, self).__init__(updater, config, nodename)
        self.box.setShown(self.state)
        self.sig_node_state_change.connect(self.box.setShown)


class TabGroup(GroupWidget):
    def __init__(self, parent, updater, config, nodename):
        super(TabGroup, self).__init__(updater, config, nodename)
        self.parent = parent

        if not self.parent.tab_bar:
            self.parent.tab_bar = QTabWidget()

        self.wid = QWidget()
        self.wid.setLayout(self.grid)

        parent.tab_bar.addTab(self.wid, self.param_name)

    def display(self, grid):
        if not self.parent.tab_bar_shown:
            grid.addRow(self.parent.tab_bar)
            self.parent.tab_bar_shown = True

    def close(self):
        super(TabGroup, self).close()
        self.parent.tab_bar = None
        self.parent.tab_bar_shown = False


class ApplyGroup(BoxGroup):
    class ApplyUpdater:
        def __init__(self, updater, loopback):
            self.updater = updater
            self.loopback = loopback
            self._configs_pending = {}

        def update(self, config):
            for name, value in config.items():
                self._configs_pending[name] = value
            self.loopback(config)

        def apply_update(self):
            self.updater.update(self._configs_pending)
            self._configs_pending = {}

    def __init__(self, updater, config, nodename):
        self.updater = ApplyGroup.ApplyUpdater(updater, self.update_group)
        super(ApplyGroup, self).__init__(self.updater, config, nodename)

        self.button = QPushButton('Apply %s' % self.param_name)
        self.button.clicked.connect(self.updater.apply_update)

        self.grid.addRow(self.button)
