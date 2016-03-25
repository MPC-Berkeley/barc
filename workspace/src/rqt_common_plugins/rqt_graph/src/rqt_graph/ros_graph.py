# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
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

from __future__ import division
import os
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal
from python_qt_binding.QtGui import QCompleter, QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget
from python_qt_binding.QtSvg import QSvgGenerator

import rosgraph.impl.graph
import rosservice
import rostopic

from qt_dotgraph.dot_to_qt import DotToQtGenerator
# pydot requires some hacks
from qt_dotgraph.pydotfactory import PydotFactory
from rqt_gui_py.plugin import Plugin
# TODO: use pygraphviz instead, but non-deterministic layout will first be resolved in graphviz 2.30
# from qtgui_plugin.pygraphvizfactory import PygraphvizFactory

from .dotcode import RosGraphDotcodeGenerator, NODE_NODE_GRAPH, NODE_TOPIC_ALL_GRAPH, NODE_TOPIC_GRAPH
from .interactive_graphics_view import InteractiveGraphicsView


class RepeatedWordCompleter(QCompleter):
    """A completer that completes multiple times from a list"""
    def init(self, parent=None):
        QCompleter.init(self, parent)

    def pathFromIndex(self, index):
        path = QCompleter.pathFromIndex(self, index)
        lst = str(self.widget().text()).split(',')
        if len(lst) > 1:
            path = '%s, %s' % (','.join(lst[:-1]), path)
        return path

    def splitPath(self, path):
        path = str(path.split(',')[-1]).lstrip(' ')
        return [path]


class NamespaceCompletionModel(QAbstractListModel):
    """Ros package and stacknames"""
    def __init__(self, linewidget, topics_only):
        super(NamespaceCompletionModel, self).__init__(linewidget)
        self.names = []

    def refresh(self, names):
        namesset = set()
        for n in names:
            namesset.add(str(n).strip())
            namesset.add("-%s" % (str(n).strip()))
        self.names = sorted(namesset)

    def rowCount(self, parent):
        return len(self.names)

    def data(self, index, role):
        if index.isValid() and (role == Qt.DisplayRole or role == Qt.EditRole):
            return self.names[index.row()]
        return None


class RosGraph(Plugin):

    _deferred_fit_in_view = Signal()

    def __init__(self, context):
        super(RosGraph, self).__init__(context)
        self.initialized = False
        self.setObjectName('RosGraph')

        self._graph = None
        self._current_dotcode = None

        self._widget = QWidget()

        # factory builds generic dotcode items
        self.dotcode_factory = PydotFactory()
        # self.dotcode_factory = PygraphvizFactory()
        # generator builds rosgraph
        self.dotcode_generator = RosGraphDotcodeGenerator()
        # dot_to_qt transforms into Qt elements using dot layout
        self.dot_to_qt = DotToQtGenerator()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_graph'), 'resource', 'RosGraph.ui')
        loadUi(ui_file, self._widget, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self._widget.setObjectName('RosGraphUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)

        self._widget.graph_type_combo_box.insertItem(0, self.tr('Nodes only'), NODE_NODE_GRAPH)
        self._widget.graph_type_combo_box.insertItem(1, self.tr('Nodes/Topics (active)'), NODE_TOPIC_GRAPH)
        self._widget.graph_type_combo_box.insertItem(2, self.tr('Nodes/Topics (all)'), NODE_TOPIC_ALL_GRAPH)
        self._widget.graph_type_combo_box.setCurrentIndex(0)
        self._widget.graph_type_combo_box.currentIndexChanged.connect(self._refresh_rosgraph)

        self.node_completionmodel = NamespaceCompletionModel(self._widget.filter_line_edit, False)
        completer = RepeatedWordCompleter(self.node_completionmodel, self)
        completer.setCompletionMode(QCompleter.PopupCompletion)
        completer.setWrapAround(True)
        completer.setCaseSensitivity(Qt.CaseInsensitive)
        self._widget.filter_line_edit.editingFinished.connect(self._refresh_rosgraph)
        self._widget.filter_line_edit.setCompleter(completer)

        self.topic_completionmodel = NamespaceCompletionModel(self._widget.topic_filter_line_edit, False)
        topic_completer = RepeatedWordCompleter(self.topic_completionmodel, self)
        topic_completer.setCompletionMode(QCompleter.PopupCompletion)
        topic_completer.setWrapAround(True)
        topic_completer.setCaseSensitivity(Qt.CaseInsensitive)
        self._widget.topic_filter_line_edit.editingFinished.connect(self._refresh_rosgraph)
        self._widget.topic_filter_line_edit.setCompleter(topic_completer)

        self._widget.namespace_cluster_check_box.clicked.connect(self._refresh_rosgraph)
        self._widget.actionlib_check_box.clicked.connect(self._refresh_rosgraph)
        self._widget.dead_sinks_check_box.clicked.connect(self._refresh_rosgraph)
        self._widget.leaf_topics_check_box.clicked.connect(self._refresh_rosgraph)
        self._widget.quiet_check_box.clicked.connect(self._refresh_rosgraph)

        self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.refresh_graph_push_button.pressed.connect(self._update_rosgraph)

        self._widget.highlight_connections_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.auto_fit_graph_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.fit_in_view_push_button.setIcon(QIcon.fromTheme('zoom-original'))
        self._widget.fit_in_view_push_button.pressed.connect(self._fit_in_view)

        self._widget.load_dot_push_button.setIcon(QIcon.fromTheme('document-open'))
        self._widget.load_dot_push_button.pressed.connect(self._load_dot)
        self._widget.save_dot_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self._widget.save_dot_push_button.pressed.connect(self._save_dot)
        self._widget.save_as_svg_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self._widget.save_as_svg_push_button.pressed.connect(self._save_svg)
        self._widget.save_as_image_push_button.setIcon(QIcon.fromTheme('image'))
        self._widget.save_as_image_push_button.pressed.connect(self._save_image)

        self._update_rosgraph()
        self._deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()

        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('graph_type_combo_box_index', self._widget.graph_type_combo_box.currentIndex())
        instance_settings.set_value('filter_line_edit_text', self._widget.filter_line_edit.text())
        instance_settings.set_value('topic_filter_line_edit_text', self._widget.topic_filter_line_edit.text())
        instance_settings.set_value('namespace_cluster_check_box_state', self._widget.namespace_cluster_check_box.isChecked())
        instance_settings.set_value('actionlib_check_box_state', self._widget.actionlib_check_box.isChecked())
        instance_settings.set_value('dead_sinks_check_box_state', self._widget.dead_sinks_check_box.isChecked())
        instance_settings.set_value('leaf_topics_check_box_state', self._widget.leaf_topics_check_box.isChecked())
        instance_settings.set_value('quiet_check_box_state', self._widget.quiet_check_box.isChecked())
        instance_settings.set_value('auto_fit_graph_check_box_state', self._widget.auto_fit_graph_check_box.isChecked())
        instance_settings.set_value('highlight_connections_check_box_state', self._widget.highlight_connections_check_box.isChecked())

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.graph_type_combo_box.setCurrentIndex(int(instance_settings.value('graph_type_combo_box_index', 0)))
        self._widget.filter_line_edit.setText(instance_settings.value('filter_line_edit_text', '/'))
        self._widget.topic_filter_line_edit.setText(instance_settings.value('topic_filter_line_edit_text', '/'))
        self._widget.namespace_cluster_check_box.setChecked(instance_settings.value('namespace_cluster_check_box_state', True) in [True, 'true'])
        self._widget.actionlib_check_box.setChecked(instance_settings.value('actionlib_check_box_state', True) in [True, 'true'])
        self._widget.dead_sinks_check_box.setChecked(instance_settings.value('dead_sinks_check_box_state', True) in [True, 'true'])
        self._widget.leaf_topics_check_box.setChecked(instance_settings.value('leaf_topics_check_box_state', True) in [True, 'true'])
        self._widget.quiet_check_box.setChecked(instance_settings.value('quiet_check_box_state', True) in [True, 'true'])
        self._widget.auto_fit_graph_check_box.setChecked(instance_settings.value('auto_fit_graph_check_box_state', True) in [True, 'true'])
        self._widget.highlight_connections_check_box.setChecked(instance_settings.value('highlight_connections_check_box_state', True) in [True, 'true'])
        self.initialized = True
        self._refresh_rosgraph()

    def _update_rosgraph(self):
        # re-enable controls customizing fetched ROS graph
        self._widget.graph_type_combo_box.setEnabled(True)
        self._widget.filter_line_edit.setEnabled(True)
        self._widget.topic_filter_line_edit.setEnabled(True)
        self._widget.namespace_cluster_check_box.setEnabled(True)
        self._widget.actionlib_check_box.setEnabled(True)
        self._widget.dead_sinks_check_box.setEnabled(True)
        self._widget.leaf_topics_check_box.setEnabled(True)
        self._widget.quiet_check_box.setEnabled(True)

        self._graph = rosgraph.impl.graph.Graph()
        self._graph.set_master_stale(5.0)
        self._graph.set_node_stale(5.0)
        self._graph.update()
        self.node_completionmodel.refresh(self._graph.nn_nodes)
        self.topic_completionmodel.refresh(self._graph.nt_nodes)
        self._refresh_rosgraph()

    def _refresh_rosgraph(self):
        if not self.initialized:
            return
        self._update_graph_view(self._generate_dotcode())

    def _generate_dotcode(self):
        ns_filter = self._widget.filter_line_edit.text()
        topic_filter = self._widget.topic_filter_line_edit.text()
        graph_mode = self._widget.graph_type_combo_box.itemData(self._widget.graph_type_combo_box.currentIndex())
        orientation = 'LR'
        if self._widget.namespace_cluster_check_box.isChecked():
            namespace_cluster = 1
        else:
            namespace_cluster = 0
        accumulate_actions = self._widget.actionlib_check_box.isChecked()
        hide_dead_end_topics = self._widget.dead_sinks_check_box.isChecked()
        hide_single_connection_topics = self._widget.leaf_topics_check_box.isChecked()
        quiet = self._widget.quiet_check_box.isChecked()

        return self.dotcode_generator.generate_dotcode(
            rosgraphinst=self._graph,
            ns_filter=ns_filter,
            topic_filter=topic_filter,
            graph_mode=graph_mode,
            hide_single_connection_topics=hide_single_connection_topics,
            hide_dead_end_topics=hide_dead_end_topics,
            cluster_namespaces_level=namespace_cluster,
            accumulate_actions=accumulate_actions,
            dotcode_factory=self.dotcode_factory,
            orientation=orientation,
            quiet=quiet)

    def _update_graph_view(self, dotcode):
        if dotcode == self._current_dotcode:
            return
        self._current_dotcode = dotcode
        self._redraw_graph_view()

    def _generate_tool_tip(self, url):
        if url is not None and ':' in url:
            item_type, item_path = url.split(':', 1)
            if item_type == 'node':
                tool_tip = 'Node:\n  %s' % (item_path)
                service_names = rosservice.get_service_list(node=item_path)
                if service_names:
                    tool_tip += '\nServices:'
                    for service_name in service_names:
                        try:
                            service_type = rosservice.get_service_type(service_name)
                            tool_tip += '\n  %s [%s]' % (service_name, service_type)
                        except rosservice.ROSServiceIOException as e:
                            tool_tip += '\n  %s' % (e)
                return tool_tip
            elif item_type == 'topic':
                topic_type, topic_name, _ = rostopic.get_topic_type(item_path)
                return 'Topic:\n  %s\nType:\n  %s' % (topic_name, topic_type)
        return url

    def _redraw_graph_view(self):
        self._scene.clear()

        if self._widget.highlight_connections_check_box.isChecked():
            highlight_level = 3
        else:
            highlight_level = 1

        # layout graph and create qt items
        (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode,
                                                            highlight_level=highlight_level,
                                                            same_label_siblings=True)

        for node_item in nodes.itervalues():
            self._scene.addItem(node_item)
        for edge_items in edges.itervalues():
            for edge_item in edge_items:
                edge_item.add_to_scene(self._scene)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())
        if self._widget.auto_fit_graph_check_box.isChecked():
            self._fit_in_view()

    def _load_dot(self, file_name=None):
        if file_name is None:
            file_name, _ = QFileDialog.getOpenFileName(self._widget, self.tr('Open graph from file'), None, self.tr('DOT graph (*.dot)'))
            if file_name is None or file_name == '':
                return

        try:
            fh = open(file_name, 'rb')
            dotcode = fh.read()
            fh.close()
        except IOError:
            return

        # disable controls customizing fetched ROS graph
        self._widget.graph_type_combo_box.setEnabled(False)
        self._widget.filter_line_edit.setEnabled(False)
        self._widget.topic_filter_line_edit.setEnabled(False)
        self._widget.namespace_cluster_check_box.setEnabled(False)
        self._widget.actionlib_check_box.setEnabled(False)
        self._widget.dead_sinks_check_box.setEnabled(False)
        self._widget.leaf_topics_check_box.setEnabled(False)
        self._widget.quiet_check_box.setEnabled(False)

        self._update_graph_view(dotcode)

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def _save_dot(self):
        file_name, _ = QFileDialog.getSaveFileName(self._widget, self.tr('Save as DOT'), 'rosgraph.dot', self.tr('DOT graph (*.dot)'))
        if file_name is None or file_name == '':
            return

        handle = QFile(file_name)
        if not handle.open(QIODevice.WriteOnly | QIODevice.Text):
            return

        handle.write(self._current_dotcode)
        handle.close()

    def _save_svg(self):
        file_name, _ = QFileDialog.getSaveFileName(self._widget, self.tr('Save as SVG'), 'rosgraph.svg', self.tr('Scalable Vector Graphic (*.svg)'))
        if file_name is None or file_name == '':
            return

        generator = QSvgGenerator()
        generator.setFileName(file_name)
        generator.setSize((self._scene.sceneRect().size() * 2.0).toSize())

        painter = QPainter(generator)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()

    def _save_image(self):
        file_name, _ = QFileDialog.getSaveFileName(self._widget, self.tr('Save as image'), 'rosgraph.png', self.tr('Image (*.bmp *.jpg *.png *.tiff)'))
        if file_name is None or file_name == '':
            return

        img = QImage((self._scene.sceneRect().size() * 2.0).toSize(), QImage.Format_ARGB32_Premultiplied)
        painter = QPainter(img)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()
        img.save(file_name)
