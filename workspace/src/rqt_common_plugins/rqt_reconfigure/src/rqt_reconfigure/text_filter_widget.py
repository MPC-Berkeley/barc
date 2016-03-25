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
from python_qt_binding.QtGui import QWidget


class TextFilterWidget(QWidget):
    """
    Taken from rqt_console.TextFilterWidget. Only modification from it is .ui
    file in use that takes more generic form (only textfiedl).
    """
    def __init__(self, parentfilter, rospack, display_list_args=None):
        """
        Widget for displaying interactive data related to text filtering.

        Taken from rqt_console and simplified to be usable in broader
        situations.

        :type parentfilter: BaseFilter
        :param parentfilter: buddy filter were data is stored, ''TimeFilter''
        :param display_list_args: empty list, ''list''
        """
        super(TextFilterWidget, self).__init__()
        ui_file = os.path.join(rospack.get_path('rqt_reconfigure'), 'resource',
                               'text_filter_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('TextFilterWidget')
        # When data is changed it is stored in the parent filter
        self._parentfilter = parentfilter

        self.text_edit.textChanged.connect(self.handle_text_changed)

        self.handle_text_changed()

    def set_text(self, text):
        """
        Setter for the text edit widget
        :param text: text to be placed in text_edit, ''str''
        """
        self.text_edit.setText(text)

    def handle_text_changed(self):
        self._parentfilter.set_text(self.text_edit.text())

    def repopulate(self):
        """
        Stub function.
        If the widget had any dynamically adjustable data it would requery it
        in this function.
        """
        pass

    def save_settings(self, settings):
        settings.set_value('text', self._parentfilter._text)

    def restore_settings(self, settings):
        text = settings.value('text', '')
        self.set_text(text)
        self.handle_text_changed()
