#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

import unittest
import rospkg

import os
from rqt_graph.dotcode import RosGraphDotcodeGenerator

PKG='rqt_graph'

class DotcodeTest(unittest.TestCase):

    def test_split_filter_empty(self):
        gen = RosGraphDotcodeGenerator()
        inc, exc = gen._split_filter_string('')
        self.assertEqual(['.*'], inc)
        self.assertEqual(0, len(exc))
        inc, exc = gen._split_filter_string('/')
        self.assertEqual(['.*'], inc)
        self.assertEqual(0, len(exc))


    def test_split_filter_includes(self):
        gen = RosGraphDotcodeGenerator()
        inc, exc = gen._split_filter_string('foo')
        self.assertEqual(['foo'], inc)
        self.assertEqual(0, len(exc))
        inc, exc = gen._split_filter_string('foo,bar')
        self.assertEqual(['foo', 'bar'], inc)
        self.assertEqual(0, len(exc))

    def test_split_filter_excludes(self):
        gen = RosGraphDotcodeGenerator()
        inc, exc = gen._split_filter_string('-foo')
        self.assertEqual(['.*'], inc)
        self.assertEqual(['foo'], exc)
        inc, exc = gen._split_filter_string('-foo,-bar')
        self.assertEqual(['.*'], inc)
        self.assertEqual(['foo', 'bar'], exc)

    def test_split_filter_both(self):
        gen = RosGraphDotcodeGenerator()
        inc, exc = gen._split_filter_string('-foo , bar ,baz, -bam')
        self.assertEqual(['bar', 'baz'], inc)
        self.assertEqual(['foo', 'bam'], exc)

    class MockEdge():
        def __init__(self, start, end):
            self.start = start
            self.end = end

    def test_get_node_edge_map(self):
        gen = RosGraphDotcodeGenerator()
        e1 = self.MockEdge('foo', 'bar')
        nmap = gen._get_node_edge_map([e1])
        self.assertEqual(2, len(nmap))
        self.assertTrue('foo' in nmap)
        self.assertTrue('bar' in nmap)
        self.assertEqual([], nmap['foo'].incoming)
        self.assertEqual([e1], nmap['foo'].outgoing)
        self.assertEqual([e1], nmap['bar'].incoming)
        self.assertEqual([], nmap['bar'].outgoing)

    def test_get_node_edge_map(self):
        gen = RosGraphDotcodeGenerator()
        e1 = self.MockEdge('foo', 'bar')
        e2 = self.MockEdge('bar', 'foo')
        e3 = self.MockEdge('foo', 'pam')
        nmap = gen._get_node_edge_map([e1, e2, e3])
        self.assertEqual(3, len(nmap))
        self.assertTrue('foo' in nmap)
        self.assertTrue('bar' in nmap)
        self.assertTrue('pam' in nmap)
        self.assertEqual([e2], nmap['foo'].incoming)
        self.assertEqual([e1, e3], nmap['foo'].outgoing)
        self.assertEqual([e1], nmap['bar'].incoming)
        self.assertEqual([e2], nmap['bar'].outgoing)

    def test_filter_leaf_topics_single_connection(self):
        gen = RosGraphDotcodeGenerator()
        topic_nodes = ['foo', 'bar', 'pam', 'boo']
        e1 = self.MockEdge('n1', 'foo')
        e2 = self.MockEdge('n2', 'foo')
        e3 = self.MockEdge('n3', 'bar')
        e4 = self.MockEdge('bar', 'n4')
        e5 = self.MockEdge('n5', 'pam')
        e6 = self.MockEdge('boo', 'n6')
        edges = [e1, e2, e3, e4, e5, e6]
        node_connections = gen._get_node_edge_map(edges)
        print(node_connections)
        rnodes, redges = gen._filter_leaf_topics(topic_nodes,
                                edges,
                                node_connections,
                                True, #hide_single_connection_topics,
                                False #hide_dead_end_topics
                                )
        self.assertEqual(['foo', 'bar'], rnodes)
        self.assertEqual([e1, e2, e3, e4], redges)
        self.assertEqual(['foo', 'bar', 'pam', 'boo'], topic_nodes)
        self.assertEqual([e1, e2, e3, e4, e5, e6], edges)

    def test_filter_leaf_topics_dead_end(self):
        gen = RosGraphDotcodeGenerator()
        topic_nodes = ['foo', 'bar', 'pam', 'boo']
        e1 = self.MockEdge('n1', 'foo')
        e2 = self.MockEdge('n2', 'foo')
        e3 = self.MockEdge('n3', 'bar')
        e4 = self.MockEdge('bar', 'n4')
        e5 = self.MockEdge('n5', 'pam')
        e6 = self.MockEdge('boo', 'n6')
        edges = [e1, e2, e3, e4, e5, e6]
        node_connections = gen._get_node_edge_map(edges)
        print(node_connections)
        rnodes, redges = gen._filter_leaf_topics(topic_nodes,
                                edges,
                                node_connections,
                                False, #hide_single_connection_topics,
                                True #hide_dead_end_topics
                                )
        self.assertEqual(['bar', 'boo'], rnodes)
        self.assertEqual([e3, e4, e6], redges)
        self.assertEqual(['foo', 'bar', 'pam', 'boo'], topic_nodes)
        self.assertEqual([e1, e2, e3, e4, e5, e6], edges)


    def test_filter_leaf_topics_both(self):
        gen = RosGraphDotcodeGenerator()
        topic_nodes = ['foo', 'bar', 'pam', 'boo']
        e1 = self.MockEdge('n1', 'foo')
        e2 = self.MockEdge('n2', 'foo')
        e3 = self.MockEdge('n3', 'bar')
        e4 = self.MockEdge('bar', 'n4')
        e5 = self.MockEdge('n5', 'pam')
        e6 = self.MockEdge('boo', 'n6')
        edges = [e1, e2, e3, e4, e5, e6]
        node_connections = gen._get_node_edge_map(edges)
        print(node_connections)
        rnodes, redges = gen._filter_leaf_topics(topic_nodes,
                                edges,
                                node_connections,
                                True, #hide_single_connection_topics,
                                True #hide_dead_end_topics
                                )
        self.assertEqual(['bar'], rnodes)
        self.assertEqual([e3, e4], redges)
        self.assertEqual(['foo', 'bar', 'pam', 'boo'], topic_nodes)
        self.assertEqual([e1, e2, e3, e4, e5, e6], edges)

    def test_accumulate_action_topics(self):
        gen = RosGraphDotcodeGenerator()
        topic_nodes = ['foo/feedback', 'foo/goal', 'foo/cancel', 'foo/result', 'foo/status', 'bar']
        e1 = self.MockEdge('n1', 'foo/feedback')
        e2 = self.MockEdge('n1', 'foo/goal')
        e3 = self.MockEdge('n1', 'foo/cancel')
        e4 = self.MockEdge('n1', 'foo/result')
        e5 = self.MockEdge('n1', 'foo/status')
        e6 = self.MockEdge('n2', 'bar')
        edges = [e1, e2, e3, e4, e5, e6]
        node_connections = gen._get_node_edge_map(edges)
        rnodes, redges, raction_nodes = gen._accumulate_action_topics(topic_nodes, edges, node_connections)
        self.assertEqual(1, len(rnodes)) # node 'bar'
        self.assertEqual(1, len(redges))
        self.assertEqual(1, len(raction_nodes))
        self.assertEqual(['foo/feedback', 'foo/goal', 'foo/cancel', 'foo/result', 'foo/status', 'bar'], topic_nodes)
        self.assertEqual([e1, e2, e3, e4, e5, e6], edges)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'dotcode_test', DotcodeTest)
