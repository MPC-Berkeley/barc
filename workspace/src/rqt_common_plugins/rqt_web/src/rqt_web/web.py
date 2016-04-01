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

from qt_gui.plugin import Plugin

from web_widget import WebWidget


class Web(Plugin):
    """
    Plugin to interface with webtools via ros_gui
    """
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(Web, self).__init__(context)
        self.setObjectName('Web')

        #  This method is used to allow user to type a url into the url bar
        self._web = WebWidget()
        if context.serial_number() > 1:
            self._web.setWindowTitle(self._web.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._web)

        #  This method is used to specify a static url
        #  NOTE: this method will hide the url bar
        #self._web = WebWidget('http://ros.org')
        #context.add_widget(self._web)

        #  To change the url at a later time use this function
        # self._web.set_url('http://willowgarage.com')

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # NOTE: This line is required to save the url bar autocompleter data between sessions
        self._web.save_settings(instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        # NOTE: This line is required to restore the url bar autocompleter data between sessions
        self._web.restore_settings(instance_settings)
