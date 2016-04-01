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
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import (QAction, QIcon, QMenu, QMessageBox,
                                     QTreeView, QWidget)
import roslib
import rosmsg
import rospkg
import rospy


from .messages_tree_view import MessagesTreeView
from rqt_py_common import rosaction
from rqt_console.text_browse_dialog import TextBrowseDialog


class MessagesWidget(QWidget):
    """
    This class is intended to be able to handle msg, srv & action (actionlib).
    The name of the class is kept to use message, by following the habit of
    rosmsg (a script that can handle both msg & srv).
    """
    def __init__(self, mode=rosmsg.MODE_MSG,
                 pkg_name='rqt_msg',
                 ui_filename='messages.ui'):
        """
        :param ui_filename: This Qt-based .ui file must have elements that are
                            referred from this class. Otherwise unexpected
                            errors are likely to happen. Best way to avoid that
                            situation when you want to give your own .ui file
                            is to implement all Qt components in
                            rqt_msg/resource/message.ui file.
        """

        super(MessagesWidget, self).__init__()
        self._rospack = rospkg.RosPack()
        ui_file = os.path.join(self._rospack.get_path(pkg_name), 'resource', ui_filename)
        loadUi(ui_file, self, {'MessagesTreeView': MessagesTreeView})
        self.setObjectName(ui_filename)
        self._mode = mode

        self._add_button.setIcon(QIcon.fromTheme('list-add'))
        self._add_button.clicked.connect(self._add_message)
        self._refresh_packages(mode)
        self._refresh_msgs(self._package_combo.itemText(0))
        self._package_combo.currentIndexChanged[str].connect(self._refresh_msgs)
        self._messages_tree.mousePressEvent = self._handle_mouse_press

        self._browsers = []

    def _refresh_packages(self, mode=rosmsg.MODE_MSG):
        if (self._mode == rosmsg.MODE_MSG) or self._mode == rosmsg.MODE_SRV:
            packages = sorted([pkg_tuple[0] for pkg_tuple in
                               rosmsg.iterate_packages(self._rospack, self._mode)])
        elif self._mode == rosaction.MODE_ACTION:
            packages = sorted([pkg_tuple[0]
                               for pkg_tuple in rosaction.iterate_packages(
                                                         self._rospack, self._mode)])
        self._package_list = packages
        rospy.logdebug('pkgs={}'.format(self._package_list))
        self._package_combo.clear()
        self._package_combo.addItems(self._package_list)
        self._package_combo.setCurrentIndex(0)

    def _refresh_msgs(self, package=None):
        if package is None or len(package) == 0:
            return
        self._msgs = []
        if (self._mode == rosmsg.MODE_MSG or
            self._mode == rosaction.MODE_ACTION):
            msg_list = rosmsg.list_msgs(package)
        elif self._mode == rosmsg.MODE_SRV:
            msg_list = rosmsg.list_srvs(package)

        rospy.logdebug('_refresh_msgs package={} msg_list={}'.format(package,
                                                                    msg_list))
        for msg in msg_list:
            if (self._mode == rosmsg.MODE_MSG or
                self._mode == rosaction.MODE_ACTION):
                msg_class = roslib.message.get_message_class(msg)
            elif self._mode == rosmsg.MODE_SRV:
                msg_class = roslib.message.get_service_class(msg)

            rospy.logdebug('_refresh_msgs msg_class={}'.format(msg_class))

            if msg_class is not None:
                self._msgs.append(msg)

        self._msgs = [x.split('/')[1] for x in self._msgs]

        self._msgs_combo.clear()
        self._msgs_combo.addItems(self._msgs)

    def _add_message(self):
        if self._msgs_combo.count() == 0:
            return
        msg = (self._package_combo.currentText() +
               '/' + self._msgs_combo.currentText())

        rospy.logdebug('_add_message msg={}'.format(msg))

        if (self._mode == rosmsg.MODE_MSG or
            self._mode == rosaction.MODE_ACTION):
            msg_class = roslib.message.get_message_class(msg)()
            if self._mode == rosmsg.MODE_MSG:
                text_tree_root = 'Msg Root'
            elif self._mode == rosaction.MODE_ACTION:
                text_tree_root = 'Action Root'
            self._messages_tree.model().add_message(msg_class,
                                            self.tr(text_tree_root), msg, msg)

        elif self._mode == rosmsg.MODE_SRV:
            msg_class = roslib.message.get_service_class(msg)()
            self._messages_tree.model().add_message(msg_class._request_class,
                                                self.tr('Service Request'),
                                                msg, msg)
            self._messages_tree.model().add_message(msg_class._response_class,
                                                self.tr('Service Response'),
                                                msg, msg)
        self._messages_tree._recursive_set_editable(
                        self._messages_tree.model().invisibleRootItem(), False)

    def _handle_mouse_press(self, event,
                            old_pressEvent=QTreeView.mousePressEvent):
        if (event.buttons() & Qt.RightButton and
            event.modifiers() == Qt.NoModifier):
            self._rightclick_menu(event)
            event.accept()
        return old_pressEvent(self._messages_tree, event)

    def _rightclick_menu(self, event):
        """
        :type event: QEvent
        """

        # QTreeview.selectedIndexes() returns 0 when no node is selected.
        # This can happen when after booting no left-click has been made yet
        # (ie. looks like right-click doesn't count). These lines are the
        # workaround for that problem.
        selected = self._messages_tree.selectedIndexes()
        if len(selected) == 0:
            return

        menu = QMenu()
        text_action = QAction(self.tr('View Text'), menu)
        menu.addAction(text_action)
        raw_action = QAction(self.tr('View Raw'), menu)
        menu.addAction(raw_action)

        action = menu.exec_(event.globalPos())

        if action == raw_action or action == text_action:
            rospy.logdebug('_rightclick_menu selected={}'.format(selected))
            selected_type = selected[1].data()

            if selected_type[-2:] == '[]':
                selected_type = selected_type[:-2]
            browsetext = None
            try:
                if (self._mode == rosmsg.MODE_MSG or
                    self._mode == rosaction.MODE_ACTION):
                    browsetext = rosmsg.get_msg_text(selected_type,
                                                     action == raw_action)
                elif self._mode == rosmsg.MODE_SRV:
                    browsetext = rosmsg.get_srv_text(selected_type,
                                                     action == raw_action)

                else:
                    raise
            except rosmsg.ROSMsgException, e:
                QMessageBox.warning(self, self.tr('Warning'),
                                    self.tr('The selected item component ' +
                                            'does not have text to view.'))
            if browsetext is not None:
                self._browsers.append(TextBrowseDialog(browsetext,
                                                       self._rospack))
                self._browsers[-1].show()
        else:
            return

    def cleanup_browsers_on_close(self):
        for browser in self._browsers:
            browser.close()
