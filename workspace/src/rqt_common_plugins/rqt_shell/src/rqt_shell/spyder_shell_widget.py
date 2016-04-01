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

from python_qt_binding.QtGui import QFont, QIcon
from python_qt_binding.QtCore import QProcess, SIGNAL, QTextCodec, Signal

from spyderlib.widgets.externalshell.baseshell import ExternalShellBase
from spyderlib.widgets.shell import TerminalWidget


class SpyderShellWidget(ExternalShellBase):
    """Spyder Shell Widget: execute a shell in a separate process using spyderlib's ExternalShellBase"""
    SHELL_CLASS = TerminalWidget
    close_signal = Signal()

    def __init__(self, parent=None):
        ExternalShellBase.__init__(self, parent=parent, fname=None, wdir='.',
                                   history_filename='.history',
                                   light_background=True,
                                   menu_actions=None,
                                   show_buttons_inside=False,
                                   show_elapsed_time=False)

        self.setObjectName('SpyderShellWidget')

        # capture tab key
        #self.shell._key_tab = self._key_tab

        self.shell.set_pythonshell_font(QFont('Mono'))

        # Additional python path list
        self.path = []

        # For compatibility with the other shells that can live in the external console
        self.is_ipython_kernel = False
        self.connection_file = None

        self.create_process()

    def get_icon(self):
        return QIcon()

    def create_process(self):
        self.shell.clear()

        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)

        env = [unicode(key_val_pair) for key_val_pair in self.process.systemEnvironment()]
        env.append('TERM=xterm')
        env.append('COLORTERM=gnome-terminal')
        self.process.setEnvironment(env)

        # Working directory
        if self.wdir is not None:
            self.process.setWorkingDirectory(self.wdir)

        self.process.readyReadStandardOutput.connect(self.write_output)
        self.process.finished.connect(self.finished)
        self.process.finished.connect(self.close_signal)

        self.process.start('/bin/bash', ['-i'])

        running = self.process.waitForStarted()
        self.set_running_state(running)
        if not running:
            self.shell.addPlainText("Process failed to start")
        else:
            self.shell.setFocus()
            self.emit(SIGNAL('started()'))

        return self.process

    def shutdown(self):
        self.process.kill()
        self.process.waitForFinished()

    def _key_tab(self):
        self.process.write('\t')
        self.process.waitForBytesWritten(-1)
        self.write_output()

    def send_to_process(self, text):
        if not isinstance(text, basestring):
            text = unicode(text)
        if not text.endswith('\n'):
            text += '\n'
        self.process.write(QTextCodec.codecForLocale().fromUnicode(text))
        self.process.waitForBytesWritten(-1)

    def keyboard_interrupt(self):
        self.send_ctrl_to_process('c')
