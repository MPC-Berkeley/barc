#!/usr/bin/env python
# Copyright (c) 2013, Oregon State University
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Oregon State University nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL OREGON STATE UNIVERSITY BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author Dan Lazewatsky/lazewatd@engr.orst.edu

from __future__ import division

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QLabel, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QCheckBox, QWidget, QToolBar, QLineEdit, QPushButton
from python_qt_binding.QtCore import Qt, QTimer

from rqt_top.node_info import NodeInfo
import re
from threading import RLock
import textwrap


class TopWidgetItem(QTreeWidgetItem):
    def __init__(self, parent=None):
        super(TopWidgetItem, self).__init__(parent)
        
    def __lt__(self, other):
        col = self.treeWidget().sortColumn()
        dtype = Top.SORT_TYPE[col]
        return dtype(self.text(col)) < dtype(other.text(col))


class Top(Plugin):

    NODE_FIELDS   = [             'pid', 'get_cpu_percent', 'get_memory_percent', 'get_num_threads']
    OUT_FIELDS    = ['node_name', 'pid', 'cpu_percent',     'memory_percent',     'num_threads'    ]
    FORMAT_STRS   = ['%s',        '%s',  '%0.2f',           '%0.2f',              '%s'             ]
    NODE_LABELS   = ['Node',      'PID', 'CPU %',           'Mem %',              'Num Threads'    ]
    SORT_TYPE     = [str,         str,   float,             float,                float            ]
    TOOLTIPS      = {
        0: ('cmdline', lambda x: '\n'.join(textwrap.wrap(' '.join(x)))),
        3: ('memory_info', lambda x: ('Resident: %0.2f MiB, Virtual: %0.2f MiB' % (x[0]/2**20, x[1]/2**20)))
    }

    _node_info = NodeInfo()

    name_filter = re.compile('')

    def __init__(self, context):
        super(Top, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Top')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
        #     print 'arguments: ', args
        #     print 'unknowns: ', unknowns

        self._selected_node = ''
        self._selected_node_lock = RLock()

        # Setup the toolbar
        self._toolbar = QToolBar()
        self._filter_box = QLineEdit()
        self._regex_box  = QCheckBox()
        self._regex_box.setText('regex')
        self._toolbar.addWidget(QLabel('Filter'))
        self._toolbar.addWidget(self._filter_box)
        self._toolbar.addWidget(self._regex_box)

        self._filter_box.returnPressed.connect(self.update_filter)
        self._regex_box.stateChanged.connect(self.update_filter)

        # Create a container widget and give it a layout
        self._container = QWidget()
        self._container.setWindowTitle('Process Monitor')
        self._layout    = QVBoxLayout()
        self._container.setLayout(self._layout)

        self._layout.addWidget(self._toolbar)

        # Create the table widget
        self._table_widget = QTreeWidget()
        self._table_widget.setObjectName('TopTable')
        self._table_widget.setColumnCount(len(self.NODE_LABELS))
        self._table_widget.setHeaderLabels(self.NODE_LABELS)
        self._table_widget.itemClicked.connect(self._tableItemClicked)
        self._table_widget.setSortingEnabled(True)
        self._table_widget.setAlternatingRowColors(True)

        self._layout.addWidget(self._table_widget)
        context.add_widget(self._container)

        # Add a button for killing nodes
        self._kill_button = QPushButton('Kill Node')
        self._layout.addWidget(self._kill_button)
        self._kill_button.clicked.connect(self._kill_node)

        # Update twice since the first cpu% lookup will always return 0
        self.update_table()
        self.update_table()

        self._table_widget.resizeColumnToContents(0)

        # Start a timer to trigger updates
        self._update_timer = QTimer()
        self._update_timer.setInterval(1000)
        self._update_timer.timeout.connect(self.update_table)
        self._update_timer.start()

    def _tableItemClicked(self, item, column):
        with self._selected_node_lock:
            self._selected_node = item.text(0)

    def update_filter(self, *args):
        if self._regex_box.isChecked():
            expr = self._filter_box.text()
        else:
            expr = re.escape(self._filter_box.text())
        self.name_filter = re.compile(expr)
        self.update_table()

    def _kill_node(self):
        self._node_info.kill_node(self._selected_node)

    def update_one_item(self, row, info):
        twi = TopWidgetItem()
        for col, field in enumerate(self.OUT_FIELDS):
            val = info[field]
            twi.setText(col, self.FORMAT_STRS[col] % val)
        self._table_widget.insertTopLevelItem(row, twi)

        for col, (key, func) in self.TOOLTIPS.iteritems():
            twi.setToolTip(col, func(info[key]))

        with self._selected_node_lock:
            if twi.text(0) == self._selected_node:
                twi.setSelected(True)

        self._table_widget.setItemHidden(twi, len(self.name_filter.findall(info['node_name'])) == 0)

    def update_table(self):
        self._table_widget.clear()
        infos = self._node_info.get_all_node_fields(self.NODE_FIELDS)
        for nx, info in enumerate(infos):
            self.update_one_item(nx, info)

    def shutdown_plugin(self):
        self._update_timer.stop()

    def save_settings(self, plugin_settings, instance_settings):        
        instance_settings.set_value('filter_text', self._filter_box.text())
        instance_settings.set_value('is_regex', int(self._regex_box.checkState()))

    def restore_settings(self, plugin_settings, instance_settings):
        self._filter_box.setText(instance_settings.value('filter_text'))
        is_regex_int = instance_settings.value('is_regex')
        if is_regex_int:
            self._regex_box.setCheckState(Qt.CheckState(is_regex_int))
        else:
            self._regex_box.setCheckState(Qt.CheckState(0))
        self.update_filter()

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog
