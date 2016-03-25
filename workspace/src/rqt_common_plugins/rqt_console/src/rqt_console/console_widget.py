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

import os

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QApplication, QCursor, QFileDialog, QHeaderView,QIcon, QMenu, QMessageBox, QTableView, QWidget
from python_qt_binding.QtCore import QRegExp, Qt, qWarning

import time
import datetime

from rqt_py_common.ini_helper import pack, unpack

from .filters.custom_filter import CustomFilter
from .filters.location_filter import LocationFilter
from .filters.message_filter import MessageFilter
from .filters.node_filter import NodeFilter
from .filters.severity_filter import SeverityFilter
from .filters.time_filter import TimeFilter
from .filters.topic_filter import TopicFilter

from .filters.custom_filter_widget import CustomFilterWidget
from .filters.filter_wrapper_widget import FilterWrapperWidget
from .filters.list_filter_widget import ListFilterWidget
from .filters.text_filter_widget import TextFilterWidget
from .filters.time_filter_widget import TimeFilterWidget

from .message import Message
from .message_data_model import MessageDataModel

from .text_browse_dialog import TextBrowseDialog


class ConsoleWidget(QWidget):
    """
    Primary widget for the rqt_console plugin.
    """
    def __init__(self, proxy_model, rospack, minimal=False):
        """
        :param proxymodel: the proxy model to display in the widget,''QSortFilterProxyModel''
        :param minimal: if true the load, save and column buttons will be hidden as well as the filter splitter, ''bool''
        """
        super(ConsoleWidget, self).__init__()
        self._proxy_model = proxy_model
        self._model = self._proxy_model.sourceModel()
        self._paused = False
        self._rospack = rospack

        # These are lists of Tuples = (,)
        self._exclude_filters = []
        self._highlight_filters = []

        ui_file = os.path.join(self._rospack.get_path('rqt_console'), 'resource', 'console_widget.ui')
        loadUi(ui_file, self)

        if minimal:
            self.load_button.hide()
            self.save_button.hide()
            self.column_resize_button.hide()
        self.setObjectName('ConsoleWidget')
        self.table_view.setModel(proxy_model)

        self._columnwidth = (60, 100, 70, 100, 100, 100, 100)
        for idx, width in enumerate(self._columnwidth):
            self.table_view.horizontalHeader().resizeSection(idx, width)

        def update_sort_indicator(logical_index, order):
            if logical_index == 0:
                self._proxy_model.sort(-1)
            self.table_view.horizontalHeader().setSortIndicatorShown(logical_index != 0)
        self.table_view.horizontalHeader().sortIndicatorChanged.connect(update_sort_indicator)

        self.add_exclude_button.setIcon(QIcon.fromTheme('list-add'))
        self.add_highlight_button.setIcon(QIcon.fromTheme('list-add'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        if not self.pause_button.icon().isNull():
            self.pause_button.setText('')
        self.record_button.setIcon(QIcon.fromTheme('media-record'))
        if not self.record_button.icon().isNull():
            self.record_button.setText('')
        self.load_button.setIcon(QIcon.fromTheme('document-open'))
        if not self.load_button.icon().isNull():
            self.load_button.setText('')
        self.save_button.setIcon(QIcon.fromTheme('document-save'))
        if not self.save_button.icon().isNull():
            self.save_button.setText('')
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        if not self.clear_button.icon().isNull():
            self.clear_button.setText('')
        self.highlight_exclude_button.setIcon(QIcon.fromTheme('format-text-strikethrough'))

        self.pause_button.clicked[bool].connect(self._handle_pause_clicked)
        self.record_button.clicked[bool].connect(self._handle_record_clicked)
        self.load_button.clicked[bool].connect(self._handle_load_clicked)
        self.save_button.clicked[bool].connect(self._handle_save_clicked)
        self.column_resize_button.clicked[bool].connect(self._handle_column_resize_clicked)
        self.clear_button.clicked[bool].connect(self._handle_clear_button_clicked)

        self.table_view.mouseDoubleClickEvent = self._handle_mouse_double_click
        self.table_view.mousePressEvent = self._handle_mouse_press
        self.table_view.keyPressEvent = self._handle_custom_keypress

        self.highlight_exclude_button.clicked[bool].connect(self._proxy_model.set_show_highlighted_only)

        self.add_highlight_button.clicked.connect(self._add_highlight_filter)
        self.add_exclude_button.clicked.connect(self._add_exclude_filter)

        # Filter factory dictionary:
        # index 0 is a label describing the widget, index 1 is the class that provides filtering logic
        # index 2 is the widget that sets the data in the filter class, index 3 are the arguments for the widget class constructor
        self._filter_factory_order = ['message', 'severity', 'node', 'time', 'topic', 'location', 'custom']
        self.filter_factory = {'message': (self.tr('...containing'), MessageFilter, TextFilterWidget),
                               'severity': (self.tr('...with severities'), SeverityFilter, ListFilterWidget, self._model.get_severity_dict),
                               'node': (self.tr('...from node'), NodeFilter, ListFilterWidget, self._model.get_unique_nodes),
                               'time': (self.tr('...from time range'), TimeFilter, TimeFilterWidget, self.get_time_range_from_selection),
                               'topic': (self.tr('...from topic'), TopicFilter, ListFilterWidget, self._model.get_unique_topics),
                               'location': (self.tr('...from location'), LocationFilter, TextFilterWidget),
                               'custom': (self.tr('Custom'), CustomFilter, CustomFilterWidget, [self._model.get_severity_dict, self._model.get_unique_nodes, self._model.get_unique_topics])}

        self._model.rowsInserted.connect(self.update_status)
        self._model.rowsRemoved.connect(self.update_status)
        self._proxy_model.rowsInserted.connect(self.update_status)
        self._proxy_model.rowsRemoved.connect(self.update_status)

        # list of TextBrowserDialogs to close when cleaning up
        self._browsers = []

        # This defaults the filters panel to start by taking 50% of the available space
        if minimal:
            self.table_splitter.setSizes([1, 0])
        else:
            self.table_splitter.setSizes([1, 1])
        self.exclude_table.resizeColumnsToContents()
        self.highlight_table.resizeColumnsToContents()

    def get_message_summary(self, start_time_offset=None, end_time_offset=None):
        """
        :param start_time: number of seconds before now to start, ''int'' (optional)
        :param end_time: number of seconds before now to end, ''int'' (optional)
        :returns: summary of message numbers within time
        """
        current_time = time.mktime(datetime.datetime.now().timetuple())
        if start_time_offset is None:
            start_time = current_time - 240
        else:
            start_time = current_time - start_time_offset
        if end_time_offset is not None:
            end_time = current_time - end_time_offset
        else:
            end_time = None

        message_subset = self._model.get_message_between(start_time, end_time)

        class Message_Summary(object):
            __slots__ = 'fatal', 'error', 'warn', 'info', 'debug'

            def __init__(self, messages):
                self.fatal = 0
                self.error = 0
                self.warn = 0
                self.info = 0
                self.debug = 0
                for message in messages:
                    if message.severity == Message.DEBUG:
                        self.debug += 1
                    elif message.severity == Message.INFO:
                        self.info += 1
                    elif message.severity == Message.WARN:
                        self.warn += 1
                    elif message.severity == Message.ERROR:
                        self.error += 1
                    elif message.severity == Message.FATAL:
                        self.fatal += 1
                    else:
                        assert False, "Unknown severity type '%s'" % str(message.severity)

        return Message_Summary(message_subset)

    def get_time_range_from_selection(self):
        """
        :returns: the range of time of messages in the current table selection (min, max), ''tuple(str,str)''
        """
        rowlist = []
        indexes = self.table_view.selectionModel().selectedIndexes()

        if indexes:
            rowlist = [self._proxy_model.mapToSource(current).row() for current in indexes]
            rowlist = sorted(list(set(rowlist)))

            mintime, maxtime = self._model.get_time_range(rowlist)
            return (mintime, maxtime)
        return (-1, -1)

    def _delete_highlight_filter(self):
        """
        Deletes any highlight filters which have a checked delete button
        """
        for index, item in enumerate(self._highlight_filters):
            if item[1].delete_button.isChecked():
                self._proxy_model.delete_highlight_filter(index)
                self.highlight_table.removeCellWidget(index, 0)
                self.highlight_table.removeRow(index)
                item[0].filter_changed_signal.disconnect(self._proxy_model.handle_highlight_filters_changed)
                item[1].delete_button.clicked.disconnect(self._delete_highlight_filter)
                del self._highlight_filters[index]

    def _delete_exclude_filter(self):
        """
        Deletes any exclude filters which have a checked delete button
        """
        for index, item in enumerate(self._exclude_filters):
            if item[1].delete_button.isChecked():
                self._proxy_model.delete_exclude_filter(index)
                self.exclude_table.removeCellWidget(index, 0)
                self.exclude_table.removeRow(index)
                item[0].filter_changed_signal.disconnect(self._proxy_model.handle_exclude_filters_changed)
                item[1].delete_button.clicked.disconnect(self._delete_exclude_filter)
                del self._exclude_filters[index]

    def _add_highlight_filter(self, filter_index=False):
        """
        :param filter_index: if false then this function shows a QMenu to allow the user to choose a type of message filter. ''bool''
        OR
        :param filter_index: the index of the filter to be added, ''int''
        :return: if a filter was added then the index is returned, ''int''
        OR
        :return: if no filter was added then None is returned, ''NoneType''
        """
        if filter_index is False:
            filter_index = -1
            filter_select_menu = QMenu()
            for index in self._filter_factory_order:
                # flattens the _highlight filters list and only adds the item if it doesn't already exist
                if index in ['message', 'location'] or not self.filter_factory[index][1] in [type(item) for sublist in self._highlight_filters for item in sublist]:
                    filter_select_menu.addAction(self.filter_factory[index][0])
            action = filter_select_menu.exec_(QCursor.pos())
            if action is None:
                return
            for index in self._filter_factory_order:
                if self.filter_factory[index][0] == action.text():
                    filter_index = index
            if filter_index == -1:
                return

        index = len(self._highlight_filters)
        newfilter = self.filter_factory[filter_index][1]()
        if len(self.filter_factory[filter_index]) >= 4:
            newwidget = self.filter_factory[filter_index][2](newfilter, self._rospack, self.filter_factory[filter_index][3])
        else:
            newwidget = self.filter_factory[filter_index][2](newfilter, self._rospack)

        # pack the new filter tuple onto the filter list
        self._highlight_filters.append((newfilter, FilterWrapperWidget(newwidget, self.filter_factory[filter_index][0]), filter_index))
        self._proxy_model.add_highlight_filter(newfilter)
        newfilter.filter_changed_signal.connect(self._proxy_model.handle_highlight_filters_changed)
        self._highlight_filters[index][1].delete_button.clicked.connect(self._delete_highlight_filter)
        self._model.rowsInserted.connect(self._highlight_filters[index][1].repopulate)

        # place the widget in the proper location
        self.highlight_table.insertRow(index)
        self.highlight_table.setCellWidget(index, 0, self._highlight_filters[index][1])
        self.highlight_table.resizeColumnsToContents()
        self.highlight_table.resizeRowsToContents()
        newfilter.filter_changed_signal.emit()
        return index

    def _add_exclude_filter(self, filter_index=False):
        """
        :param filter_index: if false then this function shows a QMenu to allow the user to choose a type of message filter. ''bool''
        OR
        :param filter_index: the index of the filter to be added, ''int''
        :return: if a filter was added then the index is returned, ''int''
        OR
        :return: if no filter was added then None is returned, ''NoneType''
        """
        if filter_index is False:
            filter_index = -1
            filter_select_menu = QMenu()
            for index in self._filter_factory_order:
                # flattens the _exclude filters list and only adds the item if it doesn't already exist
                if index in ['message', 'location'] or not self.filter_factory[index][1] in [type(item) for sublist in self._exclude_filters for item in sublist]:
                    filter_select_menu.addAction(self.filter_factory[index][0])
            action = filter_select_menu.exec_(QCursor.pos())
            if action is None:
                return None
            for index in self._filter_factory_order:
                if self.filter_factory[index][0] == action.text():
                    filter_index = index
            if filter_index == -1:
                return None

        index = len(self._exclude_filters)
        newfilter = self.filter_factory[filter_index][1]()
        if len(self.filter_factory[filter_index]) >= 4:
            newwidget = self.filter_factory[filter_index][2](newfilter, self._rospack, self.filter_factory[filter_index][3])
        else:
            newwidget = self.filter_factory[filter_index][2](newfilter, self._rospack)

        # pack the new filter tuple onto the filter list
        self._exclude_filters.append((newfilter, FilterWrapperWidget(newwidget, self.filter_factory[filter_index][0]), filter_index))
        self._proxy_model.add_exclude_filter(newfilter)
        newfilter.filter_changed_signal.connect(self._proxy_model.handle_exclude_filters_changed)
        self._exclude_filters[index][1].delete_button.clicked.connect(self._delete_exclude_filter)
        self._model.rowsInserted.connect(self._exclude_filters[index][1].repopulate)

        # place the widget in the proper location
        self.exclude_table.insertRow(index)
        self.exclude_table.setCellWidget(index, 0, self._exclude_filters[index][1])
        self.exclude_table.resizeColumnsToContents()
        self.exclude_table.resizeRowsToContents()
        newfilter.filter_changed_signal.emit()
        return index

    def _process_highlight_exclude_filter(self, selection, selectiontype, exclude=False):
        """
        Modifies the relevant filters (based on selectiontype) to remove (exclude=True)
        or highlight (exclude=False) the selection from the dataset in the tableview.
        :param selection: the actual selection, ''str''
        :param selectiontype: the type of selection, ''str''
        :param exclude: If True process as an exclude filter, False process as an highlight filter, ''bool''
        """
        types = {self.tr('Node'): 2, self.tr('Topic'): 4, self.tr('Severity'): 1, self.tr('Message'): 0}
        try:
            col = types[selectiontype]
        except:
            raise RuntimeError("Bad Column name in ConsoleWidget._process_highlight_exclude_filter()")

        if col == 0:
            unique_messages = set()
            selected_indexes = self.table_view.selectionModel().selectedIndexes()
            num_selected = len(selected_indexes) / 6
            for index in range(num_selected):
                unique_messages.add(selected_indexes[num_selected * col + index].data())
            unique_messages = list(unique_messages)
            for message in unique_messages:
                message = message.replace('\\', '\\\\')
                message = message.replace('.', '\\.')
                if exclude:
                    filter_index = self._add_exclude_filter(selectiontype.lower())
                    filter_widget = self._exclude_filters[filter_index][1].findChildren(QWidget, QRegExp('.*FilterWidget.*'))[0]
                else:
                    filter_index = self._add_highlight_filter(col)
                    filter_widget = self._highlight_filters[filter_index][1].findChildren(QWidget, QRegExp('.*FilterWidget.*'))[0]
                filter_widget.set_regex(True)
                filter_widget.set_text('^' + message + '$')

        else:
            if exclude:
                # Test if the filter we are adding already exists if it does use the existing filter
                if self.filter_factory[selectiontype.lower()][1] not in [type(item) for sublist in self._exclude_filters for item in sublist]:
                    filter_index = self._add_exclude_filter(selectiontype.lower())
                else:
                    for index, item in enumerate(self._exclude_filters):
                        if type(item[0]) == self.filter_factory[selectiontype.lower()][1]:
                            filter_index = index
            else:
                # Test if the filter we are adding already exists if it does use the existing filter
                if self.filter_factory[selectiontype.lower()][1] not in [type(item) for sublist in self._highlight_filters for item in sublist]:
                    filter_index = self._add_highlight_filter(col)
                else:
                    for index, item in enumerate(self._highlight_filters):
                        if type(item[0]) == self.filter_factory[selectiontype.lower()][1]:
                            filter_index = index

            if exclude:
                filter_widget = self._exclude_filters[filter_index][1].findChildren(QWidget, QRegExp('.*FilterWidget.*'))[0]
                filter_widget.select_item(selection)
            else:
                filter_widget = self._highlight_filters[filter_index][1].findChildren(QWidget, QRegExp('.*FilterWidget.*'))[0]
                filter_widget.select_item(selection)

    def _rightclick_menu(self, event):
        """
        Dynamically builds the rightclick menu based on the unique column data
        from the passed in datamodel and then launches it modally
        :param event: the mouse event object, ''QMouseEvent''
        """
        severities = {}
        for severity, label in Message.SEVERITY_LABELS.items():
            if severity in self._model.get_unique_severities():
                severities[severity] = label
        nodes = sorted(self._model.get_unique_nodes())
        topics = sorted(self._model.get_unique_topics())

        # menutext entries turned into
        menutext = []
        menutext.append([self.tr('Exclude'), [[self.tr('Severity'), severities], [self.tr('Node'), nodes], [self.tr('Topic'), topics], [self.tr('Selected Message(s)')]]])
        menutext.append([self.tr('Highlight'), [[self.tr('Severity'), severities], [self.tr('Node'), nodes], [self.tr('Topic'), topics], [self.tr('Selected Message(s)')]]])
        menutext.append([self.tr('Copy Selected')])
        menutext.append([self.tr('Browse Selected')])

        menu = QMenu()
        submenus = []
        subsubmenus = []
        for item in menutext:
            if len(item) > 1:
                submenus.append(QMenu(item[0], menu))
                for subitem in item[1]:
                    if len(subitem) > 1:
                        subsubmenus.append(QMenu(subitem[0], submenus[-1]))
                        if isinstance(subitem[1], dict):
                            for key in sorted(subitem[1].keys()):
                                action = subsubmenus[-1].addAction(subitem[1][key])
                                action.setData(key)
                        else:
                            for subsubitem in subitem[1]:
                                subsubmenus[-1].addAction(subsubitem)
                        submenus[-1].addMenu(subsubmenus[-1])
                    else:
                        submenus[-1].addAction(subitem[0])
                menu.addMenu(submenus[-1])
            else:
                menu.addAction(item[0])
        action = menu.exec_(event.globalPos())

        if action is None or action == 0:
            return
        elif action.text() == self.tr('Browse Selected'):
            self._show_browsers()
        elif action.text() == self.tr('Copy Selected'):
            rowlist = []
            for current in self.table_view.selectionModel().selectedIndexes():
                rowlist.append(self._proxy_model.mapToSource(current).row())
            copytext = self._model.get_selected_text(rowlist)
            if copytext is not None:
                clipboard = QApplication.clipboard()
                clipboard.setText(copytext)
        elif action.text() == self.tr('Selected Message(s)'):
            if action.parentWidget().title() == self.tr('Highlight'):
                self._process_highlight_exclude_filter(action.text(), 'Message', False)
            elif action.parentWidget().title() == self.tr('Exclude'):
                self._process_highlight_exclude_filter(action.text(), 'Message', True)
            else:
                raise RuntimeError("Menu format corruption in ConsoleWidget._rightclick_menu()")
        else:
            # This processes the dynamic list entries (severity, node and topic)
            try:
                roottitle = action.parentWidget().parentWidget().title()
            except:
                raise RuntimeError("Menu format corruption in ConsoleWidget._rightclick_menu()")

            if roottitle == self.tr('Highlight'):
                self._process_highlight_exclude_filter(action.text(), action.parentWidget().title(), False)
            elif roottitle == self.tr('Exclude'):
                self._process_highlight_exclude_filter(action.text(), action.parentWidget().title(), True)
            else:
                raise RuntimeError("Unknown Root Action %s selected in ConsoleWidget._rightclick_menu()" % roottitle)

    def update_status(self):
        """
        Sets the message display label to the current value
        """
        if self._model.rowCount() == self._proxy_model.rowCount():
            tip = self.tr('Displaying %d messages') % (self._model.rowCount())
        else:
            tip = self.tr('Displaying %d of %d messages') % (self._proxy_model.rowCount(), self._model.rowCount())
        self.messages_label.setText(tip)

    def cleanup_browsers_on_close(self):
        for browser in self._browsers:
            browser.close()

    def _show_browsers(self):
        rowlist = []
        for current in self.table_view.selectionModel().selectedIndexes():
            rowlist.append(self._proxy_model.mapToSource(current).row())
        browsetext = self._model.get_selected_text(rowlist)
        if browsetext is not None:
            self._browsers.append(TextBrowseDialog(browsetext, self._rospack))
            self._browsers[-1].show()

    def _handle_clear_button_clicked(self, checked):
        self._model.remove_rows([])
        Message._next_id = 1

    def _handle_load_clicked(self, checked):
        filename = QFileDialog.getOpenFileName(self, self.tr('Load from File'), '.', self.tr('rqt_console message file {.csv} (*.csv)'))
        if filename[0] != '':
            try:
                with open(filename[0], 'r') as h:
                    lines = h.read().splitlines()
            except IOError as e:
                qWarning(str(e))
                return False

            # extract column header
            columns = lines[0].split(';')
            if len(lines) < 2:
                return True

            # join wrapped lines
            rows = []
            last_wrapped = False
            for line in lines[1:]:
                # ignore empty lines
                if not line:
                    continue
                # check for quotes and remove them
                if line == '"':
                    has_prefix = not last_wrapped
                    has_suffix = last_wrapped
                    line = ''
                else:
                    has_prefix = line[0] == '"'
                    if has_prefix:
                        line = line[1:]
                    has_suffix = line[-1] == '"'
                    if has_suffix:
                        line = line[:-1]

                # ignore line without prefix if previous line was not wrapped
                if not has_prefix and not last_wrapped:
                    continue
                # remove wrapped line which is not continued on the next line
                if last_wrapped and has_prefix:
                    rows.pop()

                # add/append lines
                if last_wrapped:
                    rows[-1] += line
                else:
                    # add line without quote prefix
                    rows.append(line)

                last_wrapped = not has_suffix

            # generate message for each row
            messages = []
            skipped = []
            for row in rows:
                data = row.split('";"')
                msg = Message()
                msg.set_stamp_format('hh:mm:ss.ZZZ (yyyy-MM-dd)')
                for i, column in enumerate(columns):
                    value = data[i]
                    if column == 'message':
                        msg.message = value.replace('\\"', '"')
                    elif column == 'severity':
                        msg.severity = int(value)
                        if msg.severity not in Message.SEVERITY_LABELS:
                            skipped.append('Unknown severity value: %s' % value)
                            msg = None
                            break
                    elif column == 'stamp':
                        parts = value.split('.')
                        if len(parts) != 2:
                            skipped.append('Unknown timestamp format: %s' % value)
                            msg = None
                            break
                        msg.stamp = (int(parts[0]), int(parts[1]))
                    elif column == 'topics':
                        msg.topics = value.split(',')
                    elif column == 'node':
                        msg.node = value
                    elif column == 'location':
                        msg.location = value
                    else:
                        skipped.append('Unknown column: %s' % column)
                        msg = None
                        break
                if msg:
                    messages.append(msg)
            if skipped:
                qWarning('Skipped %d rows since they do not appear to be in rqt_console message file format:\n- %s' % (len(skipped), '\n- '.join(skipped)))

            if messages:
                self._model.insert_rows(messages)

                self._handle_pause_clicked(True)

            return True

        else:
            qWarning('File does not appear to be a rqt_console message file: missing file header.')
            return False

    def _handle_save_clicked(self, checked):
        filename = QFileDialog.getSaveFileName(self, 'Save to File', '.', self.tr('rqt_console msg file {.csv} (*.csv)'))
        if filename[0] != '':
            filename = filename[0]
            if filename[-4:] != '.csv':
                filename += '.csv'
            try:
                handle = open(filename, 'w')
            except IOError as e:
                qWarning(str(e))
                return
            try:
                handle.write(';'.join(MessageDataModel.columns) + '\n')
                for index in range(self._proxy_model.rowCount()):
                    row = self._proxy_model.mapToSource(self._proxy_model.index(index, 0)).row()
                    msg = self._model._messages[row]
                    data = {}
                    data['message'] = msg.message.replace('"', '\\"')
                    data['severity'] = str(msg.severity)
                    data['node'] = msg.node
                    data['stamp'] = str(msg.stamp[0]) + '.' + str(msg.stamp[1]).zfill(9)
                    data['topics'] = ','.join(msg.topics)
                    data['location'] = msg.location
                    line = []
                    for column in MessageDataModel.columns:
                        line.append('"%s"' % data[column])
                    handle.write(';'.join(line) + '\n')
            except Exception as e:
                qWarning('File save failed: %s' % str(e))
                return False
            finally:
                handle.close()
            return True

    def _handle_pause_clicked(self):
        self._paused = True
        self.pause_button.setVisible(False)
        self.record_button.setVisible(True)

    def _handle_record_clicked(self):
        self._paused = False
        self.pause_button.setVisible(True)
        self.record_button.setVisible(False)

    def _handle_column_resize_clicked(self):
        self.table_view.resizeColumnsToContents()

    def _delete_selected_rows(self):
        rowlist = []
        for current in self.table_view.selectionModel().selectedIndexes():
            rowlist.append(self._proxy_model.mapToSource(current).row())
        rowlist = list(set(rowlist))
        return self._model.remove_rows(rowlist)

    def _handle_custom_keypress(self, event, old_keyPressEvent=QTableView.keyPressEvent):
        """
        Handles the delete key.
        The delete key removes the tableview's selected rows from the datamodel
        """
        if event.key() == Qt.Key_Delete and len(self._model._messages) > 0:
            delete = QMessageBox.Yes
            if len(self.table_view.selectionModel().selectedIndexes()) == 0:
                delete = QMessageBox.question(self, self.tr('Message'), self.tr("Are you sure you want to delete all messages?"), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if delete == QMessageBox.Yes and event.key() == Qt.Key_Delete and event.modifiers() == Qt.NoModifier:
                if self._delete_selected_rows():
                    event.accept()
        return old_keyPressEvent(self.table_view, event)

    def _handle_mouse_double_click(self, event, old_doubleclickevent=QTableView.mouseDoubleClickEvent):
        if event.buttons() & Qt.LeftButton and event.modifiers() == Qt.NoModifier:
            self._show_browsers()
            event.accept()
        return old_doubleclickevent(self.table_view, event)

    def _handle_mouse_press(self, event, old_pressEvent=QTableView.mousePressEvent):
        if event.buttons() & Qt.RightButton and event.modifiers() == Qt.NoModifier:
            self._rightclick_menu(event)
            event.accept()
        return old_pressEvent(self.table_view, event)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('settings_exist', True)

        instance_settings.set_value('table_splitter', self.table_splitter.saveState())
        instance_settings.set_value('filter_splitter', self.filter_splitter.saveState())

        instance_settings.set_value('paused', self._paused)
        instance_settings.set_value('show_highlighted_only', self.highlight_exclude_button.isChecked())

        exclude_filters = []
        for index, item in enumerate(self._exclude_filters):
            exclude_filters.append(item[2])
            filter_settings = instance_settings.get_settings('exclude_filter_' + str(index))
            item[1].save_settings(filter_settings)
        instance_settings.set_value('exclude_filters', pack(exclude_filters))

        highlight_filters = []
        for index, item in enumerate(self._highlight_filters):
            highlight_filters.append(item[2])
            filter_settings = instance_settings.get_settings('highlight_filter_' + str(index))
            item[1].save_settings(filter_settings)
        instance_settings.set_value('highlight_filters', pack(highlight_filters))
        instance_settings.set_value('message_limit', self._model.get_message_limit())

    def restore_settings(self, pluggin_settings, instance_settings):
        if instance_settings.contains('table_splitter'):
            self.table_splitter.restoreState(instance_settings.value('table_splitter'))
        if instance_settings.contains('filter_splitter'):
            self.filter_splitter.restoreState(instance_settings.value('filter_splitter'))
        else:
            self.filter_splitter.setSizes([1, 1])

        paused = instance_settings.value('paused') in [True, 'true']
        if paused:
            self._handle_pause_clicked()
        else:
            self._handle_record_clicked()
        self.highlight_exclude_button.setChecked(instance_settings.value('show_highlighted_only') in [True, 'true'])
        self._proxy_model.set_show_highlighted_only(self.highlight_exclude_button.isChecked())

        for item in self._exclude_filters:
            item[1].delete_button.setChecked(True)
        self._delete_exclude_filter()
        if instance_settings.contains('exclude_filters'):
            exclude_filters = unpack(instance_settings.value('exclude_filters'))
            if exclude_filters is not None:
                for index, item in enumerate(exclude_filters):
                    self._add_exclude_filter(item)
                    filter_settings = instance_settings.get_settings('exclude_filter_' + str(index))
                    self._exclude_filters[-1][1].restore_settings(filter_settings)
        else:
            self._add_exclude_filter('severity')

        for item in self._highlight_filters:
            item[1].delete_button.setChecked(True)
        self._delete_highlight_filter()
        if instance_settings.contains('highlight_filters'):
            highlight_filters = unpack(instance_settings.value('highlight_filters'))
            if highlight_filters is not None:
                for index, item in enumerate(highlight_filters):
                    self._add_highlight_filter(item)
                    filter_settings = instance_settings.get_settings('highlight_filter_' + str(index))
                    self._highlight_filters[-1][1].restore_settings(filter_settings)
        else:
            self._add_highlight_filter('message')

        if instance_settings.contains('message_limit'):
            self._model.set_message_limit(int(instance_settings.value('message_limit')))
