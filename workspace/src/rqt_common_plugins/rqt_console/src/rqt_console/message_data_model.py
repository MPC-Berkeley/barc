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

from python_qt_binding.QtCore import QAbstractTableModel, QModelIndex, Qt, qWarning
from python_qt_binding.QtGui import QBrush, QIcon

from .message import Message
from .message_list import MessageList


class MessageDataModel(QAbstractTableModel):

    # the column names must match the message attributes
    columns = ['message', 'severity', 'node', 'stamp', 'topics', 'location']

    severity_colors = {
        Message.DEBUG: QBrush(Qt.darkCyan),
        Message.INFO: QBrush(Qt.darkBlue),
        Message.WARN: QBrush(Qt.darkYellow),
        Message.ERROR: QBrush(Qt.darkRed),
        Message.FATAL: QBrush(Qt.red),
    }

    def __init__(self):
        super(MessageDataModel, self).__init__()
        self._messages = MessageList()
        self._message_limit = 20000
        self._info_icon = QIcon.fromTheme('dialog-information')
        self._warning_icon = QIcon.fromTheme('dialog-warning')
        self._error_icon = QIcon.fromTheme('dialog-error')

    # BEGIN Required implementations of QAbstractTableModel functions

    def rowCount(self, parent=None):
        return len(self._messages)

    def columnCount(self, parent=None):
        return len(MessageDataModel.columns) + 1

    def data(self, index, role=None):
        if role is None:
            role = Qt.DisplayRole
        if index.row() >= 0 and index.row() < len(self._messages):
            msg = self._messages[index.row()]
            if index.column() == 0:
                if role == Qt.DisplayRole:
                    return '#%d' % msg.id
            elif index.column() > 0 and index.column() < len(MessageDataModel.columns) + 1:
                column = MessageDataModel.columns[index.column() - 1]
                if role == Qt.DisplayRole or role == Qt.UserRole:
                    if column == 'stamp':
                        if role != Qt.UserRole:
                            data = msg.get_stamp_string()
                        else:
                            data = msg.get_stamp_for_compare()
                    else:
                        data = getattr(msg, column)
                    # map severity enum to label
                    if role == Qt.DisplayRole and column == 'severity':
                        data = Message.SEVERITY_LABELS[data]
                    # implode topic names
                    if column == 'topics':
                        data = ', '.join(data)
                    # append row number to define strict order
                    if role == Qt.UserRole:
                        # append row number to define strict order
                        # shortest string representation to compare stamps
                        #print(column, data, str(index.row()).zfill(len(str(len(self._messages)))))
                        data = str(data) + ' %08x' % index.row()
                    return data

                # decorate message column with severity icon
                if role == Qt.DecorationRole and column == 'message':
                    if msg.severity in [Message.DEBUG, Message.INFO]:
                        return self._info_icon
                    elif msg.severity in [Message.WARN]:
                        return self._warning_icon
                    elif msg.severity in [Message.ERROR, Message.FATAL]:
                        return self._error_icon

                # colorize severity label
                if role == Qt.ForegroundRole and column =='severity':
                    assert msg.severity in MessageDataModel.severity_colors, 'Unknown severity type: %s' % msg.severity
                    return MessageDataModel.severity_colors[msg.severity]

                if role == Qt.ToolTipRole and column != 'severity':
                    if column == 'stamp':
                        data = msg.get_stamp_string()
                    elif column == 'topics':
                        data = ', '.join(msg.topics)
                    else:
                        data = getattr(msg, column)
                    # <font> tag enables word wrap by forcing rich text
                    return '<font>' + data + '<br/><br/>' + self.tr('Right click for menu.') + '</font>'

    def headerData(self, section, orientation, role=None):
        if role is None:
            role = Qt.DisplayRole
        if orientation == Qt.Horizontal:
            if role == Qt.DisplayRole:
                if section == 0:
                    return self.tr('#')
                else:
                    return MessageDataModel.columns[section - 1].capitalize()
            if role == Qt.ToolTipRole:
                if section == 0:
                    return self.tr('Sort the rows by serial number in descendig order')
                else:
                    return self.tr('Sorting the table by a column other then the serial number slows down the interaction especially when recording high frequency data')

    # END Required implementations of QAbstractTableModel functions

    def get_message_limit(self):
        return self._message_limit

    def set_message_limit(self, new_limit):
        self._message_limit = new_limit
        self._enforce_message_limit(self._message_limit)

    def _enforce_message_limit(self, limit):
        if len(self._messages) > limit:
            self.beginRemoveRows(QModelIndex(), limit, len(self._messages) - 1)
            del self._messages[limit:len(self._messages)]
            self.endRemoveRows()

    def insert_rows(self, msgs):
        # never try to insert more message than the limit
        if len(msgs) > self._message_limit:
            msgs = msgs[-self._message_limit:]
        # reduce model before insert
        limit = self._message_limit - len(msgs)
        self._enforce_message_limit(limit)
        # insert newest messages
        self.beginInsertRows(QModelIndex(), 0, len(msgs) - 1)
        self._messages.extend(msgs)
        self.endInsertRows()

    def remove_rows(self, rowlist):
        """
        :param rowlist: list of row indexes, ''list(int)''
        :returns: True if the indexes were removed successfully, ''bool''
        """
        if len(rowlist) == 0:
            if len(self._messages) > 0:
                try:
                    self.beginRemoveRows(QModelIndex(), 0, len(self._messages))
                    del self._messages[0:len(self._messages)]
                    self.endRemoveRows()
                except:
                    return False
        else:
            rowlist = list(set(rowlist))
            rowlist.sort(reverse=True)
            dellist = [rowlist[0]]
            for row in rowlist[1:]:
                if dellist[-1] - 1 > row:
                    try:
                        self.beginRemoveRows(QModelIndex(), dellist[-1], dellist[0])
                        del self._messages[dellist[-1]:dellist[0] + 1]
                        self.endRemoveRows()
                    except:
                        return False
                    dellist = []
                dellist.append(row)
            if len(dellist) > 0:
                try:
                    self.beginRemoveRows(QModelIndex(), dellist[-1], dellist[0])
                    del self._messages[dellist[-1]:dellist[0] + 1]
                    self.endRemoveRows()
                except:
                    return False
        return True

    def get_selected_text(self, rowlist):
        """
        Returns an easily readable block of text for the currently selected rows
        :param rowlist: list of row indexes, ''list(int)''
        :returns: the text from those indexes, ''str''
        """
        text = None
        if len(rowlist) != 0:
            text = ''
            rowlist = list(set(rowlist))
            for row in rowlist:
                text += self._messages[row].pretty_print()
        return text

    def get_time_range(self, rowlist):
        """
        :param rowlist: a list of row indexes, ''list''
        :returns: a tuple of min and max times in a rowlist in '(unix timestamp).(fraction of second)' format, ''tuple(str,str)''
        """
        min_ = float("inf")
        max_ = float("-inf")
        for row in rowlist:
            item = self._messages[row].time_as_datestamp()
            if float(item) > float(max_):
                max_ = item
            if float(item) < float(min_):
                min_ = item
        return min_, max_

    def get_unique_nodes(self):
        nodes = set([])
        for message in self._messages:
            nodes.add(message.node)
        return nodes

    def get_unique_severities(self):
        severities = set([])
        for message in self._messages:
            severities.add(message.severity)
        return severities

    def get_unique_topics(self):
        topics = set([])
        for message in self._messages:
            for topic in message.topics:
                topics.add(topic)
        return topics

    def get_severity_dict(self):
        return Message.SEVERITY_LABELS

    def get_message_between(self, start_time, end_time=None):
        """
        :param start_time: time to start in timestamp form (including decimal
        fractions of a second is acceptable, ''unixtimestamp''
        :param end_time: time to end in timestamp form (including decimal
        fractions of a second is acceptable, ''unixtimestamp'' (Optional)
        :returns: list of messages in the time range ''list[message]''
        """
        msgs = []
        for message in self._messages:
            msg_time = message.stamp[0] + float(message.stamp[1]) / 10**9
            if msg_time >= float(start_time) and (end_time is None or msg_time <= float(end_time)):
                msgs.append(message)
        return msgs
