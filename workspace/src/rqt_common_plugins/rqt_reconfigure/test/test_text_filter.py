# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
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

import unittest

from python_qt_binding.QtCore import Qt, QRegExp
from rqt_reconfigure.text_filter import TextFilter


class MyTest(unittest.TestCase):
    _query_text = 'filter'
    _filtered_text = 'filtered_text'

    def setUp(self):
        unittest.TestCase.setUp(self)

        syntax_nr = QRegExp.RegExp
        syntax = QRegExp.PatternSyntax(syntax_nr)
        self._regExp = QRegExp(self._query_text, Qt.CaseInsensitive, syntax)

        self._filter = TextFilter(self._regExp)
        self._filter.set_text(self._query_text)

    def tearDown(self):
        unittest.TestCase.tearDown(self)

    def test_test_message(self):
        """Testing test_message method."""
        result_regex = self._filter.test_message(self._filtered_text)
        print 'result_regex={}'.format(result_regex)
        self.assertEqual(result_regex,
                         True  # Both _query_text & filtered_text overlaps.
                         )

if __name__ == '__main__':
    unittest.main()
