# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation Inc.
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
# Author: Isaac Saito

from __future__ import division

import os

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
import rospkg
import rospy


class PluginContainerWidget(QWidget):
    """
    This widget accommodates a plugin widget that needs an area to show system
    message. A plugin widget is the pane that provides plugin's main
    functionalities. PluginContainerWidget visually encapsulates a plugin
    widget.

    In order to print msg in the msg pane provided by this class, plugin widget
    MUST define and emit following signals:

    - sig_sysmsg
    - sig_progress

    Having said that this architecture is based on signals, it is recommended
    that exceptions raised in classes that are used in a plugin widget be
    aggregated in it, so that only plugin widget is responsible for emitting
    signals.
    """

    def __init__(self, plugin_widget,
                 on_sys_msg=True, on_sysprogress_bar=True):
        """
        @param plugin_widget: The main widget of an rqt plugin.
        @type plugin_widget: QWidget
        @type on_sys_msg: bool
        @param on_sys_msg: If True, a pane that accommodates str messages will
                           appear in the plugin's region.
        @param on_sysprogress_bar: If True, a progress bar will appear in the
                                   plugin's region.
        """
        super(PluginContainerWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_py_common'),
                               'resource', 'plugin_container.ui')
        loadUi(ui_file, self)

        self._plugin_widget = plugin_widget
        self._splitter.insertWidget(0, self._plugin_widget)
        self.setWindowTitle(self._plugin_widget.windowTitle())

        # Default is on for these sys status widgets. Only if the flag for them
        # are 'False', hide them.
        if on_sys_msg:
            self._plugin_widget.sig_sysmsg.connect(self.set_sysmsg)
        else:
            self._sysmsg_widget.hide()

        if on_sysprogress_bar:
            self._plugin_widget.sig_sysprogress.connect(self.set_sysprogress)
        else:
            self._sysprogress_bar.hide()

    def set_sysprogress(self, sysprogress):
        #TODO: Pass progress value
        pass

    def set_sysmsg(self, sysmsg):
        """
        Set system msg that's supposed to be shown in sys msg pane.
        @type sysmsg: str
        """
        rospy.loginfo('PluginContainerWidget; {}'.format(sysmsg))
        #self._sysmsg_widget.setPlainText(sysmsg)
        self._sysmsg_widget.append(sysmsg)

    def shutdown(self):

        #TODO: Is shutdown step necessary for PluginContainerWidget?

        self._plugin_widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):

        #Save setting of PluginContainerWidget.
        instance_settings.set_value('_splitter', self._splitter.saveState())

        #Save setting of ContainED widget
        self._plugin_widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):

        # Restore the setting of PluginContainerWidget, separate from
        # ContainED widget.
        if instance_settings.contains('_splitter'):
            self._splitter.restoreState(instance_settings.value('_splitter'))
        else:
            self._splitter.setSizes([100, 100, 200])

        # Now restore the setting of ContainED widget
        self._plugin_widget.restore_settings(plugin_settings,
                                             instance_settings)
