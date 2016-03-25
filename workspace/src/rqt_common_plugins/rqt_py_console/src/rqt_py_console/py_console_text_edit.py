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

import sys
from code import InteractiveInterpreter
from exceptions import SystemExit

from python_qt_binding import QT_BINDING, QT_BINDING_VERSION
from python_qt_binding.QtCore import Qt, Signal

from qt_gui_py_common.console_text_edit import ConsoleTextEdit


class PyConsoleTextEdit(ConsoleTextEdit):
    _color_stdin = Qt.darkGreen
    _multi_line_char = ':'
    _multi_line_indent = '    '
    _prompt = ('>>> ', '... ')  # prompt for single and multi line
    exit = Signal()

    def __init__(self, parent=None):
        super(PyConsoleTextEdit, self).__init__(parent)

        self._interpreter_locals = {}
        self._interpreter = InteractiveInterpreter(self._interpreter_locals)

        self._comment_writer.write('Python %s on %s\n' % (sys.version.replace('\n', ''), sys.platform))
        self._comment_writer.write('Qt bindings: %s version %s\n' % (QT_BINDING, QT_BINDING_VERSION))

        self._add_prompt()

    def update_interpreter_locals(self, newLocals):
        self._interpreter_locals.update(newLocals)

    def _exec_code(self, code):
        try:
            self._interpreter.runsource(code)
        except SystemExit:  # catch sys.exit() calls, so they don't close the whole gui
            self.exit.emit()
