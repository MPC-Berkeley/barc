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

from PIL import Image

# HACK workaround for upstream pillow issue python-pillow/Pillow#400
import sys
if 'PyQt5' in sys.modules:
    sys.modules['PyQt5'] = None
from PIL.ImageQt import ImageQt

from rqt_bag import TopicMessageView
import image_helper

from python_qt_binding.QtGui import QGraphicsScene, QGraphicsView, QPixmap


class ImageView(TopicMessageView):
    """
    Popup image viewer
    """
    name = 'Image'

    def __init__(self, timeline, parent, topic):
        super(ImageView, self).__init__(timeline, parent, topic)

        self._image = None
        self._image_topic = None
        self._image_stamp = None
        self.quality = Image.NEAREST  # quality hint for scaling

        # TODO put the image_topic and image_stamp on the picture or display them in some fashion
        self._overlay_font_size = 14.0
        self._overlay_indent = (4, 4)
        self._overlay_color = (0.2, 0.2, 1.0)

        self._image_view = QGraphicsView(parent)
        self._image_view.resizeEvent = self._resizeEvent
        self._scene = QGraphicsScene()
        self._image_view.setScene(self._scene)
        parent.layout().addWidget(self._image_view)

    # MessageView implementation
    def _resizeEvent(self, event):
        # TODO make this smarter. currently there will be no scrollbar even if the timeline extends beyond the viewable area
        self._scene.setSceneRect(0, 0, self._image_view.size().width() - 2, self._image_view.size().height() - 2)
        self.put_image_into_scene()

    def message_viewed(self, bag, msg_details):
        """
        refreshes the image
        """
        TopicMessageView.message_viewed(self, bag, msg_details)
        topic, msg, t = msg_details[:3]
        if not msg:
            self.set_image(None, topic, 'no message')
        else:
            self.set_image(msg, topic, msg.header.stamp)

    def message_cleared(self):
        TopicMessageView.message_cleared(self)
        self.set_image(None, None, None)

    # End MessageView implementation
    def put_image_into_scene(self):
        if self._image:
            resized_image = self._image.resize((self._image_view.size().width() - 2, self._image_view.size().height() - 2), self.quality)

            QtImage = ImageQt(resized_image)
            pixmap = QPixmap.fromImage(QtImage)
            self._scene.clear()
            self._scene.addPixmap(pixmap)

    def set_image(self, image_msg, image_topic, image_stamp):
        self._image_msg = image_msg
        if image_msg:
            self._image = image_helper.imgmsg_to_pil(image_msg)
        else:
            self._image = None
        self._image_topic = image_topic
        self._image_stamp = image_stamp
        self.put_image_into_scene()
