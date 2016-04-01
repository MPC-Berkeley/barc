#!/usr/bin/python

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

from python_qt_binding.QtGui import QStandardItem, QStandardItemModel
from rqt_reconfigure.rqt_ros_graph import RqtRosGraph


class TestRqtRosGraph(unittest.TestCase):
    """
    :author: Isaac Saito
    """

    def setUp(self):
        unittest.TestCase.setUp(self)

        self._model = QStandardItemModel()
        node1 = QStandardItem('node1')
        self._node1_1 = QStandardItem('node1_1')
        self._node1_1_1 = QStandardItem('node1_1_1')
        node1_1_2 = QStandardItem('node1_1_2')
        node1_2 = QStandardItem('node1_2')

        node1.appendRow(self._node1_1)
        self._node1_1.appendRow(self._node1_1_1)
        self._node1_1.appendRow(node1_1_2)
        node1.appendRow(node1_2)

        self._model.appendRow(node1)

        #node_list = [node1, node1_1, self._node1_1_1, node1_1_2, node1_2]
        #self._model.appendRow(node_list)

        self._grn_node1_1_1 = '/node1/node1_1/node1_1_1'
        self._len_lower_grn_node1_1 = 2

    def tearDown(self):
        unittest.TestCase.tearDown(self)
        del self._model

    def test_get_upper_grn(self):
        self.assertEqual(RqtRosGraph.get_upper_grn(self._node1_1_1.index(), ''),
                         self._grn_node1_1_1)

    def test_get_lower_grn_dfs(self):
        self.assertEqual(len(RqtRosGraph.get_lower_grn_dfs(
                                                  self._node1_1.index(),
                                                  '')),
                         self._len_lower_grn_node1_1)

    def test_get_full_grn(self):
        self.assertEqual(RqtRosGraph.get_full_grn(self._node1_1_1.index()),
                         self._grn_node1_1_1)


if __name__ == '__main__':
    unittest.main()
