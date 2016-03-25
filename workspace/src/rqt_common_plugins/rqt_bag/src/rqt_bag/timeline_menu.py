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

from python_qt_binding.QtGui import QVBoxLayout, QMenu, QWidget, QDockWidget

class TopicPopupWidget(QWidget):
    def __init__(self, popup_name, timeline, viewer_type, topic):
        super(TopicPopupWidget, self).__init__()
        self.setObjectName(popup_name)
        self.setWindowTitle(popup_name)

        layout = QVBoxLayout()
        self.setLayout(layout)

        self._timeline = timeline
        self._viewer_type = viewer_type
        self._topic = topic
        self._viewer = None
        self._is_listening = False

    def hideEvent(self, event):
        if self._is_listening:
            self._timeline.remove_listener(self._topic, self._viewer)
            self._is_listening = False
        super(TopicPopupWidget, self).hideEvent(event)

    def showEvent(self, event):
        if not self._is_listening:
            self._timeline.add_listener(self._topic, self._viewer)
            self._is_listening = True
        super(TopicPopupWidget, self).showEvent(event)

    def show(self, context):
        """
        Make this topic popup visible, if necessary. This includes setting up
        the proper close button hacks
        """
        if not self.parent():
            context.add_widget(self)
            # make the dock widget closable, even if it normally isn't
            dock_features = self.parent().features()
            dock_features |= QDockWidget.DockWidgetClosable
            self.parent().setFeatures(dock_features)

            # remove old listener
            if self._viewer:
                self._timeline.remove_listener(self._topic, self._viewer)
                self._viewer = None

            # clean out the layout
            while self.layout().count() > 0:
                item = self.layout().itemAt(0)
                self.layout().removeItem(item)

            # create a new viewer
            self._viewer = self._viewer_type(self._timeline, self, self._topic)
            if not self._is_listening:
                self._timeline.add_listener(self._topic, self._viewer)
                self._is_listening = True

        super(TopicPopupWidget, self).show()

class TimelinePopupMenu(QMenu):
    """
    Custom popup menu displayed on rightclick from timeline
    """
    def __init__(self, timeline, event, menu_topic):
        super(TimelinePopupMenu, self).__init__()

        self.parent = timeline
        self.timeline = timeline


        if menu_topic is not None:
            self.setTitle(menu_topic)
            self._menu_topic = menu_topic
        else:
            self._menu_topic = None

        self._reset_timeline = self.addAction('Reset Timeline')

        self._play_all = self.addAction('Play All Messages')
        self._play_all.setCheckable(True)
        self._play_all.setChecked(self.timeline.play_all)

        self.addSeparator()

        self._renderers = self.timeline._timeline_frame.get_renderers()
        self._thumbnail_actions = []

        # create thumbnail menu items
        if menu_topic is None:
            submenu = self.addMenu('Thumbnails...')
            self._thumbnail_show_action = submenu.addAction('Show All')
            self._thumbnail_hide_action = submenu.addAction('Hide All')
            submenu.addSeparator()

            for topic, renderer in self._renderers:
                self._thumbnail_actions.append(submenu.addAction(topic))
                self._thumbnail_actions[-1].setCheckable(True)
                self._thumbnail_actions[-1].setChecked(self.timeline._timeline_frame.is_renderer_active(topic))
        else:
            self._thumbnail_show_action = None
            self._thumbnail_hide_action = None
            for topic, renderer in self._renderers:
                if menu_topic == topic:
                    self._thumbnail_actions.append(self.addAction("Thumbnail"))
                    self._thumbnail_actions[-1].setCheckable(True)
                    self._thumbnail_actions[-1].setChecked(self.timeline._timeline_frame.is_renderer_active(topic))

        # create view menu items
        self._topic_actions = []
        self._type_actions = []
        if menu_topic is None:
            self._topics = self.timeline._timeline_frame.topics
            view_topics_menu = self.addMenu('View (by Topic)')
            for topic in self._topics:
                datatype = self.timeline.get_datatype(topic)

                # View... / topic
                topic_menu = QMenu(topic, self)
                viewer_types = self.timeline._timeline_frame.get_viewer_types(datatype)

                # View... / topic / Viewer
                for viewer_type in viewer_types:
                    tempaction = topic_menu.addAction(viewer_type.name)
                    tempaction.setData(viewer_type)
                    self._topic_actions.append(tempaction)
                view_topics_menu.addMenu(topic_menu)

            view_type_menu = self.addMenu('View (by Type)')
            self._topics_by_type = self.timeline._timeline_frame._topics_by_datatype
            for datatype in self._topics_by_type:
                # View... / datatype
                datatype_menu = QMenu(datatype, self)
                datatype_topics = self._topics_by_type[datatype]
                viewer_types = self.timeline._timeline_frame.get_viewer_types(datatype)
                for topic in [t for t in self._topics if t in datatype_topics]:   # use timeline ordering
                    topic_menu = QMenu(topic, datatype_menu)
                    # View... / datatype / topic / Viewer
                    for viewer_type in viewer_types:
                        tempaction = topic_menu.addAction(viewer_type.name)
                        tempaction.setData(viewer_type)
                        self._topic_actions.append(tempaction)
                    datatype_menu.addMenu(topic_menu)
                view_type_menu.addMenu(datatype_menu)
        else:
            view_menu = self.addMenu("View")
            datatype = self.timeline.get_datatype(menu_topic)

            viewer_types = self.timeline._timeline_frame.get_viewer_types(datatype)
            for viewer_type in viewer_types:
                tempaction = view_menu.addAction(viewer_type.name)
                tempaction.setData(viewer_type)
                self._topic_actions.append(tempaction)

        self.addSeparator()

        # create publish menu items
        self._publish_actions = []
        if menu_topic is None:
            submenu = self.addMenu('Publish...')

            self._publish_all = submenu.addAction('Publish All')
            self._publish_none = submenu.addAction('Publish None')

            submenu.addSeparator()

            for topic in self._topics:
                self._publish_actions.append(submenu.addAction(topic))
                self._publish_actions[-1].setCheckable(True)
                self._publish_actions[-1].setChecked(self.timeline.is_publishing(topic))
        else:
            self._publish_actions.append(self.addAction("Publish"))
            self._publish_actions[-1].setCheckable(True)
            self._publish_actions[-1].setChecked(self.timeline.is_publishing(menu_topic))
            self._publish_all = None
            self._publish_none = None



        action = self.exec_(event.globalPos())
        if action is not None and action != 0:
            self.process(action)

    def process(self, action):
        """
        :param action: action to execute, ''QAction''
        :raises: when it doesn't recognice the action passed in, ''Exception''
        """
        if action == self._reset_timeline:
            self.timeline._timeline_frame.reset_timeline()
        elif action == self._play_all:
            self.timeline.toggle_play_all()
        elif action == self._publish_all:
            for topic in self.timeline._timeline_frame.topics:
                if not self.timeline.start_publishing(topic):
                    break
        elif action == self._publish_none:
            for topic in self.timeline._timeline_frame.topics:
                if not self.timeline.stop_publishing(topic):
                    break
        elif action == self._thumbnail_show_action:
            self.timeline._timeline_frame.set_renderers_active(True)
        elif action == self._thumbnail_hide_action:
            self.timeline._timeline_frame.set_renderers_active(False)
        elif action in self._thumbnail_actions:
            if self._menu_topic is None:
                topic = action.text()
            else:
                topic = self._menu_topic

            if self.timeline._timeline_frame.is_renderer_active(topic):
                self.timeline._timeline_frame.set_renderer_active(topic, False)
            else:
                self.timeline._timeline_frame.set_renderer_active(topic, True)
        elif action in self._topic_actions + self._type_actions:
            if self._menu_topic is None:
                topic = action.parentWidget().title()
            else:
                topic = self._menu_topic

            popup_name = topic + '__' + action.text()
            if popup_name not in self.timeline.popups:
                frame = TopicPopupWidget(popup_name, self.timeline,
                                         action.data(), str(topic))

                self.timeline.add_view(topic, frame)
                self.timeline.popups[popup_name] = frame

            # make popup visible
            frame = self.timeline.popups[popup_name]
            frame.show(self.timeline.get_context())

        elif action in self._publish_actions:
            if self._menu_topic is None:
                topic = action.text()
            else:
                topic = self._menu_topic

            if self.timeline.is_publishing(topic):
                self.timeline.stop_publishing(topic)
            else:
                self.timeline.start_publishing(topic)
        else:
            raise Exception('Unknown action in TimelinePopupMenu.process')
