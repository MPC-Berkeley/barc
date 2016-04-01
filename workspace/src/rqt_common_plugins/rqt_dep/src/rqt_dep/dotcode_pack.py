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

from __future__ import with_statement, print_function

import os
import re

from rospkg import MANIFEST_FILE
from rospkg.common import ResourceNotFound
from qt_dotgraph.colors import get_color_for_string


def matches_any(name, patternlist):
    for pattern in patternlist:
        if name == pattern:
            return True
        if re.match("^[a-zA-Z0-9_]+$", pattern) is None:
            if re.match(pattern, name) is not None:
                return True
    return False


class RosPackageGraphDotcodeGenerator:

    def __init__(self, rospack, rosstack):
        """
        :param rospack: use rospkg.RosPack()
        :param rosstack: use rospkg.RosStack()
        """
        self.rospack = rospack
        self.rosstack = rosstack
        self.stacks = {}
        self.packages = {}
        self.package_types = {}
        self.edges = {}
        self.traversed_ancestors = {}
        self.traversed_descendants = {}
        self.last_drawargs = None
        self.last_selection = None

    def generate_dotcode(self,
                         dotcode_factory,
                         selected_names=[],
                         excludes=[],
                         depth=3,
                         with_stacks=True,
                         descendants=True,
                         ancestors=True,
                         hide_transitives=True,
                         mark_selected=True,
                         colortheme=None,
                         rank='same',  # None, same, min, max, source, sink
                         ranksep=0.2,  # vertical distance between layers
                         rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
                         simplify=True,  # remove double edges
                         force_refresh=False,
                         hide_wet=False,
                         hide_dry=False):
        """

        :param hide_transitives: if true, then dependency of children to grandchildren will be hidden if parent has same dependency
        """

        # defaults
        selected_names = filter(lambda x: x is not None and x != '', selected_names)
        excludes = filter(lambda x: x is not None and x != '', excludes)
        if selected_names is None or selected_names == []:
            selected_names = ['.*']
            self.depth = 1
        if depth is None:
            depth = -1

        # update arguments

        selection_args = {
            "dotcode_factory": dotcode_factory,
            "with_stacks": with_stacks,
            "depth": depth,
            "hide_transitives": hide_transitives,
            "selected_names": selected_names,
            "excludes": excludes,
            "ancestors": ancestors,
            "descendants": descendants,
            "hide_wet": hide_wet,
            "hide_dry": hide_dry
            }

        # if selection did not change, we need not build up the graph again
        selection_changed = False
        if self.last_selection != selection_args:
            selection_changed = True
            self.last_selection = selection_args

            self.dotcode_factory = dotcode_factory
            self.with_stacks = with_stacks
            self.depth = depth
            self.hide_transitives = hide_transitives
            self.selected_names = selected_names
            self.excludes = excludes
            self.ancestors = ancestors
            self.descendants = descendants
            self.hide_wet = hide_wet
            self.hide_dry = hide_dry

        if force_refresh or selection_changed:
            self.stacks = {}
            self.packages = {}
            self.package_types = {}
            self.edges = {}
            self.traversed_ancestors = {}
            self.traversed_descendants = {}
            # update internal graph structure
            for name in self.rospack.list():
                if matches_any(name, self.selected_names):
                    if descendants:
                        self.add_package_descendants_recursively(name)
                    if ancestors:
                        self.add_package_ancestors_recursively(name)
            for stackname in self.rosstack.list():
                if matches_any(stackname, self.selected_names):
                    manifest = self.rosstack.get_manifest(stackname)
                    if manifest.is_catkin:
                        if descendants:
                            self.add_package_descendants_recursively(stackname)
                        if ancestors:
                            self.add_package_ancestors_recursively(stackname)
                    else:
                        for package_name in self.rosstack.packages_of(stackname):
                            if descendants:
                                self.add_package_descendants_recursively(package_name)
                            if ancestors:
                                self.add_package_ancestors_recursively(package_name)

        drawing_args = {
            'dotcode_factory': dotcode_factory,
            "rank": rank,
            "rankdir": rankdir,
            "ranksep": ranksep,
            "simplify": simplify,
            "colortheme": colortheme,
            "mark_selected": mark_selected
            }

        # if selection and display args did not change, no need to generate dotcode
        display_changed = False
        if self.last_drawargs != drawing_args:
            display_changed = True
            self.last_drawargs = drawing_args

            self.dotcode_factory = dotcode_factory
            self.rank = rank
            self.rankdir = rankdir
            self.ranksep = ranksep
            self.simplify = simplify
            self.colortheme = colortheme
            self.dotcode_factory = dotcode_factory
            self.mark_selected = mark_selected

        #generate new dotcode
        if force_refresh or selection_changed or display_changed:
            self.graph = self.generate(self.dotcode_factory)
            self.dotcode = dotcode_factory.create_dot(self.graph)

        return self.dotcode

    def generate(self, dotcode_factory):
        graph = dotcode_factory.get_graph(rank=self.rank,
                                          rankdir=self.rankdir,
                                          ranksep=self.ranksep,
                                          simplify=self.simplify)
        # print("In generate", self.with_stacks, len(self.stacks), len(self.packages), len(self.edges))
        packages_in_stacks = []
        if self.with_stacks and not self.hide_dry:
            for stackname in self.stacks:
                color = None
                if self.mark_selected and not '.*' in self.selected_names and matches_any(stackname, self.selected_names):
                    color = 'tomato'
                else:
                    color = 'gray'
                    if self.colortheme is not None:
                        color = get_color_for_string(stackname)
                g = dotcode_factory.add_subgraph_to_graph(graph,
                                                          stackname,
                                                          color=color,
                                                          rank=self.rank,
                                                          rankdir=self.rankdir,
                                                          ranksep=self.ranksep,
                                                          simplify=self.simplify)

                for package_name in self.stacks[stackname]['packages']:
                    packages_in_stacks.append(package_name)
                    self._generate_package(dotcode_factory, g, package_name)

        for package_name, attributes in self.packages.iteritems():
            if package_name not in packages_in_stacks:
                self._generate_package(dotcode_factory, graph, package_name, attributes)
        for name1, name2 in self.edges.keys():
            dotcode_factory.add_edge_to_graph(graph, name1, name2)
        return graph

    def _generate_package(self, dotcode_factory, graph, package_name, attributes=None):
        if self._hide_package(package_name):
            return
        color = None
        if self.mark_selected and not '.*' in self.selected_names and matches_any(package_name, self.selected_names):
            if attributes and attributes['is_catkin']:
                color = 'red'
            else:
                color = 'tomato'
        elif attributes and not attributes['is_catkin']:
            color = 'gray'
        if attributes and 'not_found' in attributes and attributes['not_found']:
            color = 'orange'
            package_name += ' ?'
        dotcode_factory.add_node_to_graph(graph, package_name, color=color)

    def _add_stack(self, stackname):
        if stackname is None or stackname in self.stacks:
            return
        self.stacks[stackname] = {'packages': []}

    def _add_package(self, package_name, parent=None):
        """
        adds object based on package_name to self.packages
        :param parent: packagename which referenced package_name (for debugging only)
        """
        if self._hide_package(package_name):
            return
        if package_name in self.packages:
            return False

        catkin_package = self._is_package_wet(package_name)
        if catkin_package is None:
            return False
        self.packages[package_name] = {'is_catkin': catkin_package}

        if self.with_stacks:
            try:
                stackname = self.rospack.stack_of(package_name)
            except ResourceNotFound as e:
                print('RosPackageGraphDotcodeGenerator._add_package(%s), parent %s: ResourceNotFound:' % (package_name, parent), e)
                stackname = None
            if not stackname is None and stackname != '':
                if not stackname in self.stacks:
                    self._add_stack(stackname)
                self.stacks[stackname]['packages'].append(package_name)
        return True

    def _hide_package(self, package_name):
        if not self.hide_wet and not self.hide_dry:
            return False
        catkin_package = self._is_package_wet(package_name)
        if self.hide_wet and catkin_package:
            return True
        if self.hide_dry and catkin_package is False:
            return True
        # if type of package is unknown don't hide it
        return False

    def _is_package_wet(self, package_name):
        if package_name not in self.package_types:
            try:
                package_path = self.rospack.get_path(package_name)
                manifest_file = os.path.join(package_path, MANIFEST_FILE)
                self.package_types[package_name] = not os.path.exists(manifest_file)
            except ResourceNotFound:
                return None
        return self.package_types[package_name]

    def _add_edge(self, name1, name2, attributes=None):
        if self._hide_package(name1) or self._hide_package(name2):
            return
        self.edges[(name1, name2)] = attributes

    def add_package_ancestors_recursively(self, package_name, expanded_up=None, depth=None, implicit=False, parent=None):
        """
        :param package_name: the name of package for which to add ancestors
        :param expanded_up: names that have already been expanded (to avoid cycles)
        :param depth: how many layers to follow
        :param implicit: arg to rospack
        :param parent: package that referenced package_name for error message only
        """
        if package_name in self.traversed_ancestors:
            traversed_depth = self.traversed_ancestors[package_name]
            if traversed_depth is None:
                return
            if depth is not None and traversed_depth >= depth:
                return
        self.traversed_ancestors[package_name] = depth

        if matches_any(package_name, self.excludes):
            return False
        if (depth == 0):
            return False
        if (depth == None):
            depth = self.depth
        self._add_package(package_name, parent=parent)
        if expanded_up is None:
            expanded_up = []
        expanded_up.append(package_name)
        if (depth != 1):
            try:
                depends_on = self.rospack.get_depends_on(package_name, implicit=implicit)
            except ResourceNotFound as e:
                print('RosPackageGraphDotcodeGenerator.add_package_ancestors_recursively(%s), parent %s: ResourceNotFound:' % (package_name, parent), e)
                depends_on = []
            new_nodes = []
            for dep_on_name in [x for x in depends_on if not matches_any(x, self.excludes)]:
                if not self.hide_transitives or not dep_on_name in expanded_up:
                    new_nodes.append(dep_on_name)
                    self._add_edge(dep_on_name, package_name)
                    self._add_package(dep_on_name, parent=package_name)
                    expanded_up.append(dep_on_name)
            for dep_on_name in new_nodes:
                self.add_package_ancestors_recursively(package_name=dep_on_name,
                                                       expanded_up=expanded_up,
                                                       depth=depth - 1,
                                                       implicit=implicit,
                                                       parent=package_name)

    def add_package_descendants_recursively(self, package_name, expanded=None, depth=None, implicit=False, parent=None):
        if package_name in self.traversed_descendants:
            traversed_depth = self.traversed_descendants[package_name]
            if traversed_depth is None:
                return
            if depth is not None and traversed_depth >= depth:
                return
        self.traversed_descendants[package_name] = depth

        if matches_any(package_name, self.excludes):
            return
        if (depth == 0):
            return
        if (depth == None):
            depth = self.depth
        self._add_package(package_name, parent=parent)
        if expanded is None:
            expanded = []
        expanded.append(package_name)
        if (depth != 1):
            try:
                try:
                    depends = self.rospack.get_depends(package_name, implicit=implicit)
                except ResourceNotFound:
                    # try falling back to rosstack to find wet metapackages
                    manifest = self.rosstack.get_manifest(package_name)
                    if manifest.is_catkin:
                        depends = [d.name for d in manifest.depends]
                    else:
                        raise
            except ResourceNotFound as e:
                print('RosPackageGraphDotcodeGenerator.add_package_descendants_recursively(%s), parent: %s: ResourceNotFound:' % (package_name, parent), e)
                depends = []
            new_nodes = []
            for dep_name in [x for x in depends if not matches_any(x, self.excludes)]:
                if not self.hide_transitives or not dep_name in expanded:
                    new_nodes.append(dep_name)
                    self._add_edge(package_name, dep_name)
                    self._add_package(dep_name, parent=package_name)
                    expanded.append(dep_name)
            for dep_name in new_nodes:
                self.add_package_descendants_recursively(package_name=dep_name,
                                                         expanded=expanded,
                                                         depth=depth - 1,
                                                         implicit=implicit,
                                                         parent=package_name)
