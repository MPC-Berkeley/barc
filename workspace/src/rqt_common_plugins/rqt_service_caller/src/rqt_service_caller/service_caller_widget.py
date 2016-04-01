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

from __future__ import division
import math
import os
import random
import time

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot, qWarning
from python_qt_binding.QtGui import QIcon, QMenu, QTreeWidgetItem, QWidget

import rospkg
import rospy
import genpy
import rosservice

from rqt_py_common.extended_combo_box import ExtendedComboBox


class ServiceCallerWidget(QWidget):
    column_names = ['service', 'type', 'expression']

    def __init__(self):
        super(ServiceCallerWidget, self).__init__()
        self.setObjectName('ServiceCallerWidget')

        # create context for the expression eval statement
        self._eval_locals = {}
        for module in (math, random, time):
            self._eval_locals.update(module.__dict__)
        self._eval_locals['genpy'] = genpy
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_service_caller'), 'resource', 'ServiceCaller.ui')
        loadUi(ui_file, self, {'ExtendedComboBox': ExtendedComboBox})
        self.refresh_services_button.setIcon(QIcon.fromTheme('view-refresh'))
        self.call_service_button.setIcon(QIcon.fromTheme('call-start'))

        self._column_index = {}
        for column_name in self.column_names:
            self._column_index[column_name] = len(self._column_index)

        self._service_info = None
        self.on_refresh_services_button_clicked()

        self.request_tree_widget.itemChanged.connect(self.request_tree_widget_itemChanged)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('splitter_orientation', self.splitter.orientation())

    def restore_settings(self, plugin_settings, instance_settings):
        if int(instance_settings.value('splitter_orientation', Qt.Vertical)) == int(Qt.Vertical):
            self.splitter.setOrientation(Qt.Vertical)
        else:
            self.splitter.setOrientation(Qt.Horizontal)

    def trigger_configuration(self):
        new_orientation = Qt.Vertical if self.splitter.orientation() == Qt.Horizontal else Qt.Horizontal
        self.splitter.setOrientation(new_orientation)

    @Slot()
    def on_refresh_services_button_clicked(self):
        service_names = rosservice.get_service_list()
        self._services = {}
        for service_name in service_names:
            try:
                self._services[service_name] = rosservice.get_service_class_by_name(service_name)
                #qDebug('ServiceCaller.on_refresh_services_button_clicked(): found service %s using class %s' % (service_name, self._services[service_name]))
            except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
                qWarning('ServiceCaller.on_refresh_services_button_clicked(): could not get class of service %s:\n%s' % (service_name, e))

        self.service_combo_box.clear()
        self.service_combo_box.addItems(sorted(service_names))

    @Slot(str)
    def on_service_combo_box_currentIndexChanged(self, service_name):
        self.request_tree_widget.clear()
        self.response_tree_widget.clear()
        service_name = str(service_name)
        if not service_name:
            return

        self._service_info = {}
        self._service_info['service_name'] = service_name
        self._service_info['service_class'] = self._services[service_name]
        self._service_info['service_proxy'] = rospy.ServiceProxy(service_name, self._service_info['service_class'])
        self._service_info['expressions'] = {}
        self._service_info['counter'] = 0

        # recursively create widget items for the service request's slots
        request_class = self._service_info['service_class']._request_class
        top_level_item = self._recursive_create_widget_items(None, service_name, request_class._type, request_class())

        # add top level item to tree widget
        self.request_tree_widget.addTopLevelItem(top_level_item)

        # resize columns
        self.request_tree_widget.expandAll()
        for i in range(self.request_tree_widget.columnCount()):
            self.request_tree_widget.resizeColumnToContents(i)

    def _recursive_create_widget_items(self, parent, topic_name, type_name, message, is_editable=True):
        item = QTreeWidgetItem(parent)
        if is_editable:
            item.setFlags(item.flags() | Qt.ItemIsEditable)
        else:
            item.setFlags(item.flags() & (~Qt.ItemIsEditable))

        if parent is None:
            # show full topic name with preceding namespace on toplevel item
            topic_text = topic_name
        else:
            topic_text = topic_name.split('/')[-1]

        item.setText(self._column_index['service'], topic_text)
        item.setText(self._column_index['type'], type_name)

        item.setData(0, Qt.UserRole, topic_name)

        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name, type_name in zip(message.__slots__, message._slot_types):
                self._recursive_create_widget_items(item, topic_name + '/' + slot_name, type_name, getattr(message, slot_name), is_editable)

        elif type(message) in (list, tuple) and (len(message) > 0) and hasattr(message[0], '__slots__'):
            type_name = type_name.split('[', 1)[0]
            for index, slot in enumerate(message):
                self._recursive_create_widget_items(item, topic_name + '[%d]' % index, type_name, slot, is_editable)

        else:
            item.setText(self._column_index['expression'], repr(message))

        return item

    @Slot('QTreeWidgetItem*', int)
    def request_tree_widget_itemChanged(self, item, column):
        column_name = self.column_names[column]
        new_value = str(item.text(column))
        #qDebug('ServiceCaller.request_tree_widget_itemChanged(): %s : %s' % (column_name, new_value))

        if column_name == 'expression':
            topic_name = str(item.data(0, Qt.UserRole))
            self._service_info['expressions'][topic_name] = new_value
            #qDebug('ServiceCaller.request_tree_widget_itemChanged(): %s expression: %s' % (topic_name, new_value))

    def fill_message_slots(self, message, topic_name, expressions, counter):
        if not hasattr(message, '__slots__'):
            return
        for slot_name in message.__slots__:
            slot_key = topic_name + '/' + slot_name

            # if no expression exists for this slot_key, continue with it's child slots
            if slot_key not in expressions:
                self.fill_message_slots(getattr(message, slot_name), slot_key, expressions, counter)
                continue

            expression = expressions[slot_key]
            if len(expression) == 0:
                continue

            # get slot type
            slot = getattr(message, slot_name)
            if hasattr(slot, '_type'):
                slot_type = slot._type
            else:
                slot_type = type(slot)

            self._eval_locals['i'] = counter
            value = self._evaluate_expression(expression, slot_type)
            if value is not None:
                setattr(message, slot_name, value)

    def _evaluate_expression(self, expression, slot_type):
        successful_eval = True
        successful_conversion = True

        try:
            # try to evaluate expression
            value = eval(expression, {}, self._eval_locals)
        except Exception:
            # just use expression-string as value
            value = expression
            successful_eval = False

        try:
            # try to convert value to right type
            value = slot_type(value)
        except Exception:
            successful_conversion = False

        if successful_conversion:
            return value
        elif successful_eval:
            qWarning('ServiceCaller.fill_message_slots(): can not convert expression to slot type: %s -> %s' % (type(value), slot_type))
        else:
            qWarning('ServiceCaller.fill_message_slots(): failed to evaluate expression: %s' % (expression))

        return None

    @Slot()
    def on_call_service_button_clicked(self):
        self.response_tree_widget.clear()

        request = self._service_info['service_class']._request_class()
        self.fill_message_slots(request, self._service_info['service_name'], self._service_info['expressions'], self._service_info['counter'])
        try:
            response = self._service_info['service_proxy'](request)
        except rospy.ServiceException as e:
            qWarning('ServiceCaller.on_call_service_button_clicked(): request:\n%r' % (request))
            qWarning('ServiceCaller.on_call_service_button_clicked(): error calling service "%s":\n%s' % (self._service_info['service_name'], e))
            top_level_item = QTreeWidgetItem()
            top_level_item.setText(self._column_index['service'], 'ERROR')
            top_level_item.setText(self._column_index['type'], 'rospy.ServiceException')
            top_level_item.setText(self._column_index['expression'], str(e))
        else:
            #qDebug('ServiceCaller.on_call_service_button_clicked(): response: %r' % (response))
            top_level_item = self._recursive_create_widget_items(None, '/', response._type, response, is_editable=False)

        self.response_tree_widget.addTopLevelItem(top_level_item)
        # resize columns
        self.response_tree_widget.expandAll()
        for i in range(self.response_tree_widget.columnCount()):
            self.response_tree_widget.resizeColumnToContents(i)

    @Slot('QPoint')
    def on_request_tree_widget_customContextMenuRequested(self, pos):
        self._show_context_menu(self.request_tree_widget.itemAt(pos), self.request_tree_widget.mapToGlobal(pos))

    @Slot('QPoint')
    def on_response_tree_widget_customContextMenuRequested(self, pos):
        self._show_context_menu(self.response_tree_widget.itemAt(pos), self.response_tree_widget.mapToGlobal(pos))

    def _show_context_menu(self, item, global_pos):
        if item is None:
            return

        # show context menu
        menu = QMenu(self)
        action_item_expand = menu.addAction(QIcon.fromTheme('zoom-in'), "Expand All Children")
        action_item_collapse = menu.addAction(QIcon.fromTheme('zoom-out'), "Collapse All Children")
        action = menu.exec_(global_pos)

        # evaluate user action
        if action in (action_item_expand, action_item_collapse):
            expanded = (action is action_item_expand)

            def recursive_set_expanded(item):
                item.setExpanded(expanded)
                for index in range(item.childCount()):
                    recursive_set_expanded(item.child(index))
            recursive_set_expanded(item)

