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
#    contributors may be used to stoporse or promote products derived
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

from python_qt_binding.QtCore import QRegExp, Qt
from rqt_console.filters.message_filter import MessageFilter


class TextFilter(MessageFilter):
    """
    Provides a filtering feature for text set by set_text.

    Inheriting rqt_console.filters.MessageFilter, this class provides timeout
    effect to the input widget (eg. QLineEdit) that contains this class.
    """

    def __init__(self, qregexp=None):
        super(TextFilter, self).__init__()
        self._regexp = qregexp

    def test_message(self, text):
        """
        Overridden.

        :param message: the message to be tested against the filters.
        :type message: str.
        :rtype: bool
        """

        _hit = False

        if (self.is_enabled() and
            self._text != '' and
            not self._qregexp == None  # If None, init process isn't done yet
                                       # and we can ignore the call to this
                                       # method.
            ):
            pos_hit = self._qregexp.indexIn(text)
            if pos_hit >= 0:
                _hit = True
            else:
                _hit = False
        return _hit

#    def set_regexp(self, qregexp):
#        """
#        Setter for self._qregexp. Need not be used if _qregexp is already set
#        via __init__. Calling this method overwrites the existing regex
#        instance.
#
#        Do not mix up self._qregexp with MessageFilter._regex that is boolean.
#        Instead, this method sets QRegExp instance, that test_message method
#        uses.
#
#        :type qregexp: QRegExp
#        """
#        self._qregexp = qregexp

    def get_regexp(self):
        return self._regex

    def set_text(self, text):
        """
        Setter for _text
        :param text: text to set ''str''
        :emits filter_changed_signal: If _enabled is true
        """
        super(TextFilter, self).set_text(text)

        syntax_nr = QRegExp.RegExp
        syntax = QRegExp.PatternSyntax(syntax_nr)
        self.regex = QRegExp(text, Qt.CaseInsensitive, syntax)
        self.set_regex(self.regex)

    def get_text(self):
        return self._text
