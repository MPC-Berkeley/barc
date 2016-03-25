#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from python_qt_binding.QtCore import QModelIndex
from python_qt_binding.QtGui import QDoubleSpinBox, QItemDelegate


class SpinBoxDelegate(QItemDelegate):
    def __init__(self, min_value=0, max_value=100, decimals=2, *args):
        self._min = min_value
        self._max = max_value
        self._decimals = decimals
        super(SpinBoxDelegate, self).__init__(*args)

    def createEditor(self, parent, option, index):
        editor = QDoubleSpinBox(parent)
        editor.setDecimals(self._decimals)
        editor.setMaximum(self._min)
        editor.setMaximum(self._max)
        return editor


class DelegateUtil(object):
    """
    Find out the hierarchy level of the selected item.
    see: http://stackoverflow.com/a/4208240/577001

    :type model_index: QModelIndex
    :rtype: int

    :author: Isaac Saito
    """
    @staticmethod
    def _get_hierarchy_level(model_index):
        hierarchyLevel = 1
        seek_root = model_index
        while(seek_root.parent() != QModelIndex()):
            seek_root = seek_root.parent()
            hierarchyLevel += 1
        return hierarchyLevel
