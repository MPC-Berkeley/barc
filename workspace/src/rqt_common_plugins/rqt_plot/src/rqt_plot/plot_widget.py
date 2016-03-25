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

import os
import rospkg
import roslib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QAction, QIcon, QMenu, QWidget

import rospy

from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common import topic_helpers

from . rosplot import ROSData, RosPlotException

def get_plot_fields(topic_name):
    topic_type, real_topic, _ = topic_helpers.get_topic_type(topic_name)
    if topic_type is None:
        message = "topic %s does not exist" % ( topic_name )
        return [], message
    field_name = topic_name[len(real_topic)+1:]

    slot_type, is_array, array_size = roslib.msgs.parse_type(topic_type)
    field_class = roslib.message.get_message_class(slot_type)

    fields = [f for f in field_name.split('/') if f]

    for field in fields:
        # parse the field name for an array index
        try:
            field, _, field_index = roslib.msgs.parse_type(field)
        except roslib.msgs.MsgSpecException:
            message = "invalid field %s in topic %s" % ( field, real_topic )
            return [], message

        if field not in getattr(field_class, '__slots__', []):
            message = "no field %s in topic %s" % ( field_name, real_topic )
            return [], message
        slot_type = field_class._slot_types[field_class.__slots__.index(field)]
        slot_type, slot_is_array, array_size = roslib.msgs.parse_type(slot_type)
        is_array = slot_is_array and field_index is None

        field_class = topic_helpers.get_type_class(slot_type)

    if field_class in (int, float):
        if is_array:
            if array_size is not None:
                message = "topic %s is fixed-size numeric array" % ( topic_name )
                return [ "%s[%d]" % (topic_name, i) for i in range(array_size) ], message
            else:
                message = "topic %s is variable-size numeric array" % ( topic_name )
                return [], message
        else:
            message = "topic %s is numeric" % ( topic_name )
            return [ topic_name ], message
    else:
        if not roslib.msgs.is_valid_constant_type(slot_type):
            numeric_fields = []
            for i, slot in enumerate(field_class.__slots__):
                slot_type = field_class._slot_types[i]
                slot_type, is_array, array_size = roslib.msgs.parse_type(slot_type)
                slot_class = topic_helpers.get_type_class(slot_type)
                if slot_class in (int, float) and not is_array:
                    numeric_fields.append(slot)
            message = ""
            if len(numeric_fields) > 0:
                message = "%d plottable fields in %s" % ( len(numeric_fields), topic_name )
            else:
                message = "No plottable fields in %s" % ( topic_name )
            return [ "%s/%s" % (topic_name, f) for f in numeric_fields ], message
        else:
            message = "Topic %s is not numeric" % ( topic_name )
            return [], message

def is_plottable(topic_name):
    fields, message = get_plot_fields(topic_name)
    return len(fields) > 0, message

class PlotWidget(QWidget):
    _redraw_interval = 40

    def __init__(self, initial_topics=None, start_paused=False):
        super(PlotWidget, self).__init__()
        self.setObjectName('PlotWidget')

        self._initial_topics = initial_topics

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_plot'), 'resource', 'plot.ui')
        loadUi(ui_file, self)
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('list-add'))
        self.remove_topic_button.setIcon(QIcon.fromTheme('list-remove'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = None

        self.subscribe_topic_button.setEnabled(False)
        if start_paused:
            self.pause_button.setChecked(True)

        self._topic_completer = TopicCompleter(self.topic_edit)
        self._topic_completer.update_topics()
        self.topic_edit.setCompleter(self._topic_completer)

        self._start_time = rospy.get_time()
        self._rosdata = {}
        self._remove_topic_menu = QMenu()

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)

    def switch_data_plot_widget(self, data_plot):
        self.enable_timer(enabled=False)

        self.data_plot_layout.removeWidget(self.data_plot)
        if self.data_plot is not None:
            self.data_plot.close()

        self.data_plot = data_plot
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(self.autoscroll_checkbox.isChecked())

        # setup drag 'n drop
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent

        if self._initial_topics:
            for topic_name in self._initial_topics:
                self.add_topic(topic_name)
            self._initial_topics = None
        else:
            for topic_name, rosdata in self._rosdata.items():
                data_x, data_y = rosdata.next()
                self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)

        self._subscribed_topics_changed()

    @Slot('QDragEnterEvent*')
    def dragEnterEvent(self, event):
        # get topic name
        if not event.mimeData().hasText():
            if not hasattr(event.source(), 'selectedItems') or len(event.source().selectedItems()) == 0:
                qWarning('Plot.dragEnterEvent(): not hasattr(event.source(), selectedItems) or len(event.source().selectedItems()) == 0')
                return
            item = event.source().selectedItems()[0]
            topic_name = item.data(0, Qt.UserRole)
            if topic_name == None:
                qWarning('Plot.dragEnterEvent(): not hasattr(item, ros_topic_name_)')
                return
        else:
            topic_name = str(event.mimeData().text())

        # check for plottable field type
        plottable, message = is_plottable(topic_name)
        if plottable:
            event.acceptProposedAction()
        else:
            qWarning('Plot.dragEnterEvent(): rejecting: "%s"' % (message))

    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))
        self.add_topic(topic_name)

    @Slot(str)
    def on_topic_edit_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._topic_completer.update_topics()

        plottable, message = is_plottable(topic_name)
        self.subscribe_topic_button.setEnabled(plottable)
        self.subscribe_topic_button.setToolTip(message)

    @Slot()
    def on_topic_edit_returnPressed(self):
        if self.subscribe_topic_button.isEnabled():
            self.add_topic(str(self.topic_edit.text()))

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.add_topic(str(self.topic_edit.text()))

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    @Slot(bool)
    def on_autoscroll_checkbox_clicked(self, checked):
        self.data_plot.autoscroll(checked)
        if checked:
            self.data_plot.redraw()

    @Slot()
    def on_clear_button_clicked(self):
        self.clear_plot()

    def update_plot(self):
        if self.data_plot is not None:
            needs_redraw = False
            for topic_name, rosdata in self._rosdata.items():
                try:
                    data_x, data_y = rosdata.next()
                    if data_x or data_y:
                        self.data_plot.update_values(topic_name, data_x, data_y)
                        needs_redraw = True
                except RosPlotException as e:
                    qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
            if needs_redraw:
                self.data_plot.redraw()

    def _subscribed_topics_changed(self):
        self._update_remove_topic_menu()
        if not self.pause_button.isChecked():
            # if pause button is not pressed, enable timer based on subscribed topics
            self.enable_timer(self._rosdata)
        self.data_plot.redraw()

    def _update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for topic_name in sorted(self._rosdata.keys()):
            action = QAction(topic_name, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(topic_name))
            self._remove_topic_menu.addAction(action)

        if len(self._rosdata) > 1:
            all_action = QAction('All', self._remove_topic_menu)
            all_action.triggered.connect(self.clean_up_subscribers)
            self._remove_topic_menu.addAction(all_action)

        self.remove_topic_button.setMenu(self._remove_topic_menu)

    def add_topic(self, topic_name):
        topics_changed = False
        for topic_name in get_plot_fields(topic_name)[0]:
            if topic_name in self._rosdata:
                qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
                continue
            self._rosdata[topic_name] = ROSData(topic_name, self._start_time)
            if self._rosdata[topic_name].error is not None:
                qWarning(str(self._rosdata[topic_name].error))
                del self._rosdata[topic_name]
            else:
                data_x, data_y = self._rosdata[topic_name].next()
                self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)
                topics_changed = True

        if topics_changed:
            self._subscribed_topics_changed()

    def remove_topic(self, topic_name):
        self._rosdata[topic_name].close()
        del self._rosdata[topic_name]
        self.data_plot.remove_curve(topic_name)

        self._subscribed_topics_changed()

    def clear_plot(self):
        for topic_name, _ in self._rosdata.items():
            self.data_plot.clear_values(topic_name)
        self.data_plot.redraw()

    def clean_up_subscribers(self):
        for topic_name, rosdata in self._rosdata.items():
            rosdata.close()
            self.data_plot.remove_curve(topic_name)
        self._rosdata = {}

        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
