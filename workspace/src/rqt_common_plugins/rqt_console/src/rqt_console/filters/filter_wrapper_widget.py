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
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QIcon, QWidget


class FilterWrapperWidget(QWidget):
    """
    Wraps the other filter widgets to provide enable check box, delete button control and uniform labeling
    """
    def __init__(self, widget, filter_name):
        """
        :param widget: the Qwidget to wrap ''Qwidget''
        :param filter_name: the name to be placed on the label ''str''
        """
        super(FilterWrapperWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_console'), 'resource/filters', 'filter_wrapper_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('FilterWrapperWidget')
        self.delete_button.setIcon(QIcon.fromTheme('list-remove'))
        self._widget = widget

        # Replace the placeholder QWidget with the passed in object
        stretch = self.layout_frame.stretch(2)
        self.layout_frame.insertWidget(2, widget)
        self.layout_frame.setStretch(2, stretch)

        # This line sets the place holder to 0 width so it can no longer be seen
        # It is a hack since removing it causes other widgets not to function properly
        self.layout_frame.setStretch(3, 0)

        self.enabled_checkbox.clicked[bool].connect(self.enabled_callback)
        self.filter_name_label.setText(filter_name + ':')

    def enabled_callback(self, checked):
        self._widget._parentfilter.set_enabled(checked)
        self._widget.setEnabled(checked)

    def repopulate(self):
        self._widget.repopulate()

    def save_settings(self, settings):
        """
        Handles writing the enabled flag to the ini file and then passes the
        settings object to the wrapped widget

        :param settings: used to write the settings to an ini file ''qt_gui.settings.Settings''
        """
        settings.set_value('enabled', self._widget._parentfilter._enabled)
        self._widget.save_settings(settings)

    def restore_settings(self, settings):
        """
        Handles reading the enabled flag from the ini file.
        :param settings: used to read the settings to an ini file ''qt_gui.settings.Settings''
        """
        checked = settings.value('enabled') in [True, 'true']
        self.enabled_callback(checked)
        self.enabled_checkbox.setChecked(checked)
        self._widget.restore_settings(settings)
