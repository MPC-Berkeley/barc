# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Dorian Scholz
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
from qt_gui_py_common.simple_settings_dialog import SimpleSettingsDialog

from shell_widget import ShellWidget

try:
    from xterm_widget import XTermWidget, is_xterm_available
    _has_xterm = is_xterm_available()
except ImportError:
    XTermWidget = None
    _has_xterm = False

try:
    from spyder_shell_widget import SpyderShellWidget
    _has_spyderlib = True
except ImportError:
    SpyderShellWidget = None
    _has_spyderlib = False


class Shell(Plugin):
    """
    Plugin providing an interactive shell
    """
    # shell types in order of priority
    shell_types = [
        {
            'title': 'XTerm',
            'widget_class': XTermWidget,
            'description': 'Fully functional embedded XTerm (needs xterm and only works on X11).',
            'enabled': _has_xterm,
        },
        {
            'title': 'SpyderShell',
            'widget_class': SpyderShellWidget,
            'description': 'Advanced shell (needs spyderlib).',
            'enabled': _has_spyderlib,
        },
        {
            'title': 'SimpleShell',
            'widget_class': ShellWidget,
            'description': 'Simple shell for executing non-interactive finite commands.',
            'enabled': True,
        },
    ]

    def __init__(self, context):
        super(Shell, self).__init__(context)
        self._context = context
        self.setObjectName('Shell')

        self._widget = None

    def _switch_shell_widget(self):
        # check for available shell type
        while not self.shell_types[self._shell_type_index]['enabled']:
            self._shell_type_index += 1
        selected_shell = self.shell_types[self._shell_type_index]

        if self._widget is not None:
            if hasattr(self._widget, 'close_signal'):
                self._widget.close_signal.disconnect(self._context.close_plugin)
            self._context.remove_widget(self._widget)
            self._widget.close()

        self._widget = selected_shell['widget_class']()
        self._widget.setWindowTitle(selected_shell['title'])
        if self._context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % self._context.serial_number()))
        self._context.add_widget(self._widget)
        if hasattr(self._widget, 'close_signal'):
            self._widget.close_signal.connect(self._context.close_plugin)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('shell_type', self._shell_type_index)

    def restore_settings(self, plugin_settings, instance_settings):
        self._shell_type_index = int(instance_settings.value('shell_type', 0))
        self._switch_shell_widget()

    def trigger_configuration(self):
        dialog = SimpleSettingsDialog(title='Shell Options')
        dialog.add_exclusive_option_group(title='Shell Type', options=self.shell_types, selected_index=self._shell_type_index)
        shell_type = dialog.get_settings()[0]
        if shell_type is not None and self._shell_type_index != shell_type['selected_index']:
            self._shell_type_index = shell_type['selected_index']
            self._context.reload_plugin()

    def shutdown_plugin(self):
        if self._widget is not None and hasattr(self._widget, 'shutdown'):
            self._widget.shutdown()

