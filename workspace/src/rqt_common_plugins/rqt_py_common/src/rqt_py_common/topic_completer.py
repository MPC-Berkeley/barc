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

import roslib
import rospy

from python_qt_binding.QtCore import qWarning

from .message_tree_model import MessageTreeModel
from .tree_model_completer import TreeModelCompleter


class TopicCompleter(TreeModelCompleter):

    def __init__(self, parent=None):
        super(TopicCompleter, self).__init__(parent)
        self.setModel(MessageTreeModel())

    def splitPath(self, path):
        # to handle array subscriptions, e.g. /topic/field[1]/subfield[2]
        # we need to separate array subscriptions by an additional /
        return super(TopicCompleter,self).splitPath(path.replace('[','/['))

    def update_topics(self):
        self.model().clear()
        topic_list = rospy.get_published_topics()
        for topic_path, topic_type in topic_list:
            topic_name = topic_path.strip('/')
            message_class = roslib.message.get_message_class(topic_type)
            if message_class is None:
                qWarning('TopicCompleter.update_topics(): could not get message class for topic type "%s" on topic "%s"' % (topic_type, topic_path))
                continue
            message_instance = message_class()
            self.model().add_message(message_instance, topic_name, topic_type, topic_path)


if __name__ == '__main__':
    import sys
    from python_qt_binding.QtGui import QApplication, QComboBox, QLineEdit, QMainWindow, QTreeView, QVBoxLayout, QWidget
    app = QApplication(sys.argv)
    mw = QMainWindow()
    widget = QWidget(mw)
    layout = QVBoxLayout(widget)

    edit = QLineEdit()
    edit_completer = TopicCompleter(edit)
    #edit_completer.setCompletionMode(QCompleter.InlineCompletion)
    edit.setCompleter(edit_completer)

    combo = QComboBox()
    combo.setEditable(True)
    combo_completer = TopicCompleter(combo)
    #combo_completer.setCompletionMode(QCompleter.InlineCompletion)
    combo.lineEdit().setCompleter(combo_completer)

    model_tree = QTreeView()
    model_tree.setModel(combo_completer.model())
    model_tree.expandAll()
    for column in range(combo_completer.model().columnCount()):
        model_tree.resizeColumnToContents(column)

    completion_tree = QTreeView()
    completion_tree.setModel(combo_completer.completionModel())
    completion_tree.expandAll()
    for column in range(combo_completer.completionModel().columnCount()):
        completion_tree.resizeColumnToContents(column)

    layout.addWidget(model_tree)
    layout.addWidget(completion_tree)
    layout.addWidget(edit)
    layout.addWidget(combo)
    layout.setStretchFactor(model_tree, 1)
    widget.setLayout(layout)
    mw.setCentralWidget(widget)

    mw.move(300, 0)
    mw.resize(800, 900)
    mw.show()
    app.exec_()
