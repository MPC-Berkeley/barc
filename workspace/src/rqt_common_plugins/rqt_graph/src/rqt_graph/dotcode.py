# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

# this is a modified version of rx/rxgraph/src/rxgraph/dotcode.py

import re
import copy

import rosgraph.impl.graph
import roslib
import math

import rospy
import pydot

# node/node connectivity
NODE_NODE_GRAPH = 'node_node'
# node/topic connections where an actual network connection exists
NODE_TOPIC_GRAPH = 'node_topic'
# all node/topic connections, even if no actual network connection
NODE_TOPIC_ALL_GRAPH = 'node_topic_all'

QUIET_NAMES = ['/diag_agg', '/runtime_logger', '/pr2_dashboard', '/rviz', '/rosout', '/cpu_monitor', '/monitor', '/hd_monitor', '/rxloggerlevel', '/clock', '/rqt', '/statistics']

def _conv(n):
    """Convert a node name to a valid dot name, which can't contain the leading space"""
    if n.startswith(' '):
        return 't_' + n[1:]
    else:
        return 'n_' + n

def matches_any(name, patternlist):
    if patternlist is None or len(patternlist) == 0:
        return False
    for pattern in patternlist:
        if str(name).strip() == pattern:
            return True
        if re.match("^[a-zA-Z0-9_/]+$", pattern) is None:
            if re.match(str(pattern), name.strip()) is not None:
                return True
    return False


class NodeConnections:
    def __init__(self, incoming=None, outgoing=None):
        self.incoming = incoming or []
        self.outgoing = outgoing or []


class RosGraphDotcodeGenerator:

    # topic/topic -> graph.edge object
    edges = dict([])

    # ROS node name -> graph.node object
    nodes = dict([])

    def __init__(self):
        try:
            from rosgraph_msgs.msg import TopicStatistics
            self.stats_sub = rospy.Subscriber('/statistics', TopicStatistics, self.statistics_callback)
        except ImportError:
            # not supported before Indigo
            pass

    def statistics_callback(self,msg):

        # add connections (if new)
        if not msg.node_sub in self.edges:
            self.edges[msg.node_sub] = dict([])

        if not msg.topic in self.edges[msg.node_sub]:
            self.edges[msg.node_sub][msg.topic] = dict([])

        self.edges[msg.node_sub][msg.topic][msg.node_pub] = msg

    def _get_max_traffic(self):
        traffic = 10000 # start at 10kb
        for sub in self.edges:
            for topic in self.edges[sub]:
               for pub in self.edges[sub][topic]:
                    traffic = max(traffic, self.edges[sub][topic][pub].traffic)
        return traffic

    def _get_max_age(self):
        age = 0.1 # start at 100ms
        for sub in self.edges:
            for topic in self.edges[sub]:
               for pub in self.edges[sub][topic]:
                    age = max(age, self.edges[sub][topic][pub].stamp_age_mean.to_sec())
        return age

    def _get_max_age_on_topic(self, sub, topic):
        age = 0.0
        for pub in self.edges[sub][topic]:
            age = max(age, self.edges[sub][topic][pub].stamp_age_mean.to_sec())
        return age

    def _calc_edge_color(self, sub, topic, pub=None):

        age = 0.0

        if pub is None:
            age = self._get_max_age_on_topic(sub, topic)
        elif sub in self.edges and topic in self.edges[sub] and pub in self.edges[sub][topic]:
            age = self.edges[sub][topic][pub].stamp_age_mean.to_sec()

        if age == 0.0:
            return [0, 0, 0]

        # calc coloring using the age
        heat = max(age, 0) / self._get_max_age()

        # we assume that heat is normalized between 0.0 (green) and 1.0 (red)
        # 0.0->green(0,255,0) to 0.5->yellow (255,255,0) to red 1.0(255,0,0)
        if heat < 0:
            red = 0
            green = 0
        elif heat <= 0.5:
            red = int(heat * 255 * 2)
            green = 255
        elif heat > 0.5:
            red = 255
            green = 255 - int((heat - 0.5) * 255 * 2)
        else:
            red = 0
            green = 0
        return [red, green, 0]

    def _calc_edge_penwidth(self, sub, topic, pub=None):
        if pub is None and sub in self.edges and topic in self.edges[sub]:
            traffic = 0
            for p in self.edges[sub][topic]:
                if pub is None or p == pub:
                    traffic += self.edges[sub][topic][p].traffic

            # calc penwidth using the traffic in kb/s
            return int(traffic / self._get_max_traffic() * 5)
        else:
            return 1

    def _calc_statistic_info(self, sub, topic, pub=None):
        if pub is None and sub in self.edges and topic in self.edges[sub]:
            conns = len(self.edges[sub][topic])
            if conns == 1:
                pub = next(self.edges[sub][topic].iterkeys())
            else:
                penwidth = self._calc_edge_penwidth(sub,topic)
                color = self._calc_edge_color(sub,topic)
                label = "("+str(conns) + " connections)"
                return [label, penwidth, color]

        if sub in self.edges and topic in self.edges[sub] and pub in self.edges[sub][topic]:
            penwidth = self._calc_edge_penwidth(sub,topic,pub)
            color = self._calc_edge_color(sub,topic,pub)
            period = self.edges[sub][topic][pub].period_mean.to_sec()
            if period > 0.0:
                freq = str(round(1.0 / period, 1))
            else:
                freq = "?"
            age = self.edges[sub][topic][pub].stamp_age_mean.to_sec()
            age_string = ""
            if age > 0.0:
                age_string = " // " + str(round(age, 2) * 1000) + " ms"
            label = freq + " Hz" + age_string
            return [label, penwidth, color]
        else:
            return [None, None, None]

    def _add_edge(self, edge, dotcode_factory, dotgraph, is_topic=False):
        if is_topic:
            sub = edge.end
            topic = edge.label
            pub = edge.start
            [stat_label, penwidth, color] = self._calc_statistic_info(sub, topic, pub)
            if stat_label is not None:
                temp_label = edge.label + "\\n" + stat_label
                dotcode_factory.add_edge_to_graph(dotgraph, _conv(edge.start), _conv(edge.end), label=temp_label, url='topic:%s' % edge.label, penwidth=penwidth, color=color)
            else:
                dotcode_factory.add_edge_to_graph(dotgraph, _conv(edge.start), _conv(edge.end), label=edge.label, url='topic:%s' % edge.label)
        else:
            sub = edge.end.strip()
            topic = edge.start.strip()
            [stat_label, penwidth, color] = self._calc_statistic_info(sub, topic)
            if stat_label is not None:
                temp_label = edge.label + "\\n" + stat_label
                dotcode_factory.add_edge_to_graph(dotgraph, _conv(edge.start), _conv(edge.end), label=temp_label, penwidth=penwidth, color=color)
            else:
                dotcode_factory.add_edge_to_graph(dotgraph, _conv(edge.start), _conv(edge.end), label=edge.label)


    def _add_node(self, node, rosgraphinst, dotcode_factory, dotgraph, quiet):
        if node in rosgraphinst.bad_nodes:
            if quiet:
                return ''
            bn = rosgraphinst.bad_nodes[node]
            if bn.type == rosgraph.impl.graph.BadNode.DEAD:
                dotcode_factory.add_node_to_graph(dotgraph,
                                                  nodename=_conv(node),
                                                  nodelabel=node,
                                                  shape="doublecircle",
                                                  url=node,
                                                  color="red")
            else:
                dotcode_factory.add_node_to_graph(dotgraph,
                                                  nodename=_conv(node),
                                                  nodelabel=node,
                                                  shape="doublecircle",
                                                  url=node,
                                                  color="orange")
        else:
            dotcode_factory.add_node_to_graph(dotgraph,
                                              nodename=_conv(node),
                                              nodelabel=node,
                                              shape='ellipse',
                                              url=node)

    def _add_topic_node(self, node, dotcode_factory, dotgraph, quiet):
        label = rosgraph.impl.graph.node_topic(node)
        dotcode_factory.add_node_to_graph(dotgraph,
                                          nodename=_conv(node),
                                          nodelabel=label,
                                          shape='box',
                                          url="topic:%s" % label)

    def _quiet_filter(self, name):
        # ignore viewers
        for n in QUIET_NAMES:
            if n in name:
                return False
        return True

    def quiet_filter_topic_edge(self, edge):
        for quiet_label in ['/time', '/clock', '/rosout', '/statistics']:
            if quiet_label == edge.label:
                return False
        return self._quiet_filter(edge.start) and self._quiet_filter(edge.end)

    def generate_namespaces(self, graph, graph_mode, quiet=False):
        """
        Determine the namespaces of the nodes being displayed
        """
        namespaces = []
        if graph_mode == NODE_NODE_GRAPH:
            nodes = graph.nn_nodes
            if quiet:
                nodes = [n for n in nodes if not n in QUIET_NAMES]
            namespaces = list(set([roslib.names.namespace(n) for n in nodes]))

        elif graph_mode == NODE_TOPIC_GRAPH or \
                 graph_mode == NODE_TOPIC_ALL_GRAPH:
            nn_nodes = graph.nn_nodes
            nt_nodes = graph.nt_nodes
            if quiet:
                nn_nodes = [n for n in nn_nodes if not n in QUIET_NAMES]
                nt_nodes = [n for n in nt_nodes if not n in QUIET_NAMES]
            if nn_nodes or nt_nodes:
                namespaces = [roslib.names.namespace(n) for n in nn_nodes]
            # an annoyance with the rosgraph library is that it
            # prepends a space to topic names as they have to have
            # different graph node namees from nodes. we have to strip here
            namespaces.extend([roslib.names.namespace(n[1:]) for n in nt_nodes])

        return list(set(namespaces))

    def _filter_orphaned_edges(self, edges, nodes):
        nodenames = [str(n).strip() for n in nodes]
        # currently using and rule as the or rule generates orphan nodes with the current logic
        return [e for e in edges if e.start.strip() in nodenames and e.end.strip() in nodenames]

    def _filter_orphaned_topics(self, nt_nodes, edges):
        '''remove topic graphnodes without connected ROS nodes'''
        removal_nodes = []
        for n in nt_nodes:
            keep = False
            for e in edges:
                if (e.start.strip() == str(n).strip() or e.end.strip() == str(n).strip()):
                    keep = True
                    break
            if not keep:
                removal_nodes.append(n)
        for n in removal_nodes:
            nt_nodes.remove(n)
        return nt_nodes

    def _split_filter_string(self, ns_filter):
        '''splits a string after each comma, and treats tokens with leading dash as exclusions.
        Adds .* as inclusion if no other inclusion option was given'''
        includes = []
        excludes = []
        for name in ns_filter.split(','):
            if name.strip().startswith('-'):
                excludes.append(name.strip()[1:])
            else:
                includes.append(name.strip())
        if includes == [] or includes == ['/'] or includes == ['']:
            includes = ['.*']
        return includes, excludes

    def _get_node_edge_map(self, edges):
        '''returns a dict mapping node name to edge objects partitioned in incoming and outgoing edges'''
        node_connections = {}
        for edge in edges:
            if not edge.start in node_connections:
                node_connections[edge.start] = NodeConnections()
            if not edge.end in node_connections:
                node_connections[edge.end] = NodeConnections()
            node_connections[edge.start].outgoing.append(edge)
            node_connections[edge.end].incoming.append(edge)
        return node_connections

    def _filter_leaf_topics(self,
                            nodes_in,
                            edges_in,
                            node_connections,
                            hide_single_connection_topics,
                            hide_dead_end_topics):
        '''
        removes certain ending topic nodes and their edges from list of nodes and edges

        @param hide_single_connection_topics: if true removes topics that are only published/subscribed by one node
        @param hide_dead_end_topics: if true removes topics having only publishers
        '''
        if not hide_dead_end_topics and not hide_single_connection_topics:
            return nodes_in, edges_in
        # do not manipulate incoming structures
        nodes = copy.copy(nodes_in)
        edges = copy.copy(edges_in)
        removal_nodes = []
        for n in nodes:
            if n in node_connections:
                node_edges = []
                has_out_edges = False
                node_edges.extend(node_connections[n].outgoing)
                if len(node_connections[n].outgoing) > 0:
                    has_out_edges = True
                node_edges.extend(node_connections[n].incoming)
                if ((hide_single_connection_topics and len(node_edges) < 2) or
                    (hide_dead_end_topics and not has_out_edges)):
                    removal_nodes.append(n)
                    for e in node_edges:
                        if e in edges:
                            edges.remove(e)
        for n in removal_nodes:
            nodes.remove(n)
        return nodes, edges

    def _accumulate_action_topics(self, nodes_in, edges_in, node_connections):
        '''takes topic nodes, edges and node connections.
        Returns topic nodes where action topics have been removed,
        edges where the edges to action topics have been removed, and
        a map with the connection to each virtual action topic node'''
        removal_nodes = []
        action_nodes = {}
        # do not manipulate incoming structures
        nodes = copy.copy(nodes_in)
        edges = copy.copy(edges_in)
        for n in nodes:
            if str(n).endswith('/feedback'):
                prefix = str(n)[:-len('/feedback')].strip()
                action_topic_nodes = []
                action_topic_edges_out = set()
                action_topic_edges_in = set()
                for suffix in ['/status', '/result', '/goal', '/cancel', '/feedback']:
                    for n2 in nodes:
                        if str(n2).strip() == prefix + suffix:
                            action_topic_nodes.append(n2)
                            if n2 in node_connections:
                                action_topic_edges_out.update(node_connections[n2].outgoing)
                                action_topic_edges_in.update(node_connections[n2].incoming)
                if len(action_topic_nodes) == 5:
                    # found action
                    removal_nodes.extend(action_topic_nodes)
                    for e in action_topic_edges_out:
                        if e in edges:
                            edges.remove(e)
                    for e in action_topic_edges_in:
                        if e in edges:
                            edges.remove(e)
                    action_nodes[prefix] = {'topics': action_topic_nodes,
                                            'outgoing': action_topic_edges_out,
                                            'incoming': action_topic_edges_in}
        for n in removal_nodes:
            nodes.remove(n)
        return nodes, edges, action_nodes

    def generate_dotgraph(self,
                         rosgraphinst,
                         ns_filter,
                         topic_filter,
                         graph_mode,
                         dotcode_factory,
                         hide_single_connection_topics=False,
                         hide_dead_end_topics=False,
                         cluster_namespaces_level=0,
                         accumulate_actions=True,
                         orientation='LR',
                         rank='same',  # None, same, min, max, source, sink
                         ranksep=0.2,  # vertical distance between layers
                         rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
                         simplify=True,  # remove double edges
                         quiet=False):
        """
        See generate_dotcode
        """
        includes, excludes = self._split_filter_string(ns_filter)
        topic_includes, topic_excludes = self._split_filter_string(topic_filter)

        nn_nodes = []
        nt_nodes = []
        # create the node definitions
        if graph_mode == NODE_NODE_GRAPH:
            nn_nodes = rosgraphinst.nn_nodes
            nn_nodes = [n for n in nn_nodes if matches_any(n, includes) and not matches_any(n, excludes)]
            edges = rosgraphinst.nn_edges
            edges = [e for e in edges if matches_any(e.label, topic_includes) and not matches_any(e.label, topic_excludes)]

        elif graph_mode == NODE_TOPIC_GRAPH or \
                 graph_mode == NODE_TOPIC_ALL_GRAPH:
            nn_nodes = rosgraphinst.nn_nodes
            nt_nodes = rosgraphinst.nt_nodes
            nn_nodes = [n for n in nn_nodes if matches_any(n, includes) and not matches_any(n, excludes)]
            nt_nodes = [n for n in nt_nodes if matches_any(n, topic_includes) and not matches_any(n, topic_excludes)]

            # create the edge definitions, unwrap EdgeList objects into python lists
            if graph_mode == NODE_TOPIC_GRAPH:
                edges = [e for e in rosgraphinst.nt_edges]
            else:
                edges = [e for e in rosgraphinst.nt_all_edges]

        if quiet:
            nn_nodes = filter(self._quiet_filter, nn_nodes)
            nt_nodes = filter(self._quiet_filter, nt_nodes)
            if graph_mode == NODE_NODE_GRAPH:
                edges = filter(self.quiet_filter_topic_edge, edges)

        # for accumulating actions topics
        action_nodes = {}

        if graph_mode != NODE_NODE_GRAPH and (hide_single_connection_topics or hide_dead_end_topics or accumulate_actions):
            # maps outgoing and incoming edges to nodes
            node_connections = self._get_node_edge_map(edges)

            nt_nodes, edges = self._filter_leaf_topics(nt_nodes,
                                         edges,
                                         node_connections,
                                         hide_single_connection_topics,
                                         hide_dead_end_topics)

            if accumulate_actions:
                nt_nodes, edges, action_nodes = self._accumulate_action_topics(nt_nodes, edges, node_connections)

        edges = self._filter_orphaned_edges(edges, list(nn_nodes) + list(nt_nodes))
        nt_nodes = self._filter_orphaned_topics(nt_nodes, edges)

        # create the graph
        # result = "digraph G {\n  rankdir=%(orientation)s;\n%(nodes_str)s\n%(edges_str)s}\n" % vars()
        dotgraph = dotcode_factory.get_graph(rank=rank,
                                             ranksep=ranksep,
                                             simplify=simplify,
                                             rankdir=orientation)

        ACTION_TOPICS_SUFFIX = '/action_topics'
        namespace_clusters = {}
        for n in (nt_nodes or []) + [action_prefix + ACTION_TOPICS_SUFFIX for (action_prefix, _) in action_nodes.items()]:
            # cluster topics with same namespace
            if (cluster_namespaces_level > 0 and
                str(n).count('/') > 1 and
                len(str(n).split('/')[1]) > 0):
                namespace = str(n).split('/')[1]
                if namespace not in namespace_clusters:
                    namespace_clusters[namespace] = dotcode_factory.add_subgraph_to_graph(dotgraph, namespace, rank=rank, rankdir=orientation, simplify=simplify)
                self._add_topic_node(n, dotcode_factory=dotcode_factory, dotgraph=namespace_clusters[namespace], quiet=quiet)
            else:
                self._add_topic_node(n, dotcode_factory=dotcode_factory, dotgraph=dotgraph, quiet=quiet)

        # for ROS node, if we have created a namespace clusters for
        # one of its peer topics, drop it into that cluster
        if nn_nodes is not None:
            for n in nn_nodes:
                if (cluster_namespaces_level > 0 and
                    str(n).count('/') >= 1 and
                    len(str(n).split('/')[1]) > 0):
                    namespace = str(n).split('/')[1]
                    if namespace not in namespace_clusters:
                        namespace_clusters[namespace] = dotcode_factory.add_subgraph_to_graph(dotgraph, namespace, rank=rank, rankdir=orientation, simplify=simplify)
                    self._add_node(n, rosgraphinst=rosgraphinst, dotcode_factory=dotcode_factory, dotgraph=namespace_clusters[namespace], quiet=quiet)
                else:
                    self._add_node(n, rosgraphinst=rosgraphinst, dotcode_factory=dotcode_factory, dotgraph=dotgraph, quiet=quiet)

        for e in edges:
            self._add_edge(e, dotcode_factory, dotgraph=dotgraph, is_topic=(graph_mode == NODE_NODE_GRAPH))

        for (action_prefix, node_connections) in action_nodes.items():
            for out_edge in node_connections.get('outgoing', []):
                dotcode_factory.add_edge_to_graph(dotgraph, _conv(action_prefix + ACTION_TOPICS_SUFFIX), _conv(out_edge.end))
            for in_edge in node_connections.get('incoming', []):
                dotcode_factory.add_edge_to_graph(dotgraph, _conv(in_edge.start), _conv(action_prefix + ACTION_TOPICS_SUFFIX))
        return dotgraph

    def generate_dotcode(self,
                         rosgraphinst,
                         ns_filter,
                         topic_filter,
                         graph_mode,
                         dotcode_factory,
                         hide_single_connection_topics=False,
                         hide_dead_end_topics=False,
                         cluster_namespaces_level=0,
                         accumulate_actions=True,
                         orientation='LR',
                         rank='same',  # None, same, min, max, source, sink
                         ranksep=0.2,  # vertical distance between layers
                         rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
                         simplify=True,  # remove double edges
                         quiet=False):
        """
        @param rosgraphinst: RosGraph instance
        @param ns_filter: nodename filter
        @type  ns_filter: string
        @param topic_filter: topicname filter
        @type  ns_filter: string
        @param graph_mode str: NODE_NODE_GRAPH | NODE_TOPIC_GRAPH | NODE_TOPIC_ALL_GRAPH
        @type  graph_mode: str
        @param orientation: rankdir value (see ORIENTATIONS dict)
        @type  dotcode_factory: object
        @param dotcode_factory: abstract factory manipulating dot language objects
        @param hide_single_connection_topics: if true remove topics with just one connection
        @param hide_dead_end_topics: if true remove topics with publishers only
        @param cluster_namespaces_level: if > 0 places box around members of same namespace (TODO: multiple namespace layers)
        @param accumulate_actions: if true each 5 action topic graph nodes are shown as single graph node
        @return: dotcode generated from graph singleton
        @rtype: str
        """
        dotgraph = self.generate_dotgraph(rosgraphinst=rosgraphinst,
                         ns_filter=ns_filter,
                         topic_filter=topic_filter,
                         graph_mode=graph_mode,
                         dotcode_factory=dotcode_factory,
                         hide_single_connection_topics=hide_single_connection_topics,
                         hide_dead_end_topics=hide_dead_end_topics,
                         cluster_namespaces_level=cluster_namespaces_level,
                         accumulate_actions=accumulate_actions,
                         orientation=orientation,
                         rank=rank,
                         ranksep=ranksep,
                         rankdir=rankdir,
                         simplify=simplify,
                         quiet=quiet)
        dotcode = dotcode_factory.create_dot(dotgraph)
        return dotcode
