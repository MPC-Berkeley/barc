#!/usr/bin/python
# -*- coding: utf-8 -*-
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

import os

from python_qt_binding.QtCore import QProcess, QTimer, Signal
from python_qt_binding.QtGui import QX11EmbedContainer


class XTermWidget(QX11EmbedContainer):
    xterm_cmd = '/usr/bin/xterm'
    close_signal = Signal()

    def __init__(self, parent=None):
        super(XTermWidget, self).__init__(parent)
        self.setObjectName('XTermWidget')
        self._process = QProcess(self)
        self._process.finished.connect(self.close_signal)
        # let the widget finish init before embedding xterm
        QTimer.singleShot(100, self._embed_xterm)

    def _embed_xterm(self):
        args = ['-into', str(self.winId())]
        self._process.start(self.xterm_cmd, args)
        if self._process.error() == QProcess.FailedToStart:
            print "failed to execute '%s'" % self.xterm_cmd

    def shutdown(self):
        self._process.kill()
        self._process.waitForFinished()


def is_xterm_available():
    return os.path.isfile(XTermWidget.xterm_cmd)

if __name__ == '__main__':
    from PyQt4.QtGui import QApplication
    app = QApplication([])
    xt = XTermWidget()
    app.exec_()
