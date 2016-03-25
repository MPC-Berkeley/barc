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

import rospy

# HACK workaround for upstream pillow issue python-pillow/Pillow#400
import sys
if 'PyQt5' in sys.modules:
    sys.modules['PyQt5'] = None
from PIL import Image
from PIL.ImageQt import ImageQt

from rqt_bag import TimelineCache, TimelineRenderer

import image_helper

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QBrush, QPen, QPixmap


class ImageTimelineRenderer(TimelineRenderer):
    """
    Draws thumbnails of sensor_msgs/Image or sensor_msgs/CompressedImage in the timeline.
    """
    def __init__(self, timeline, thumbnail_height=160):
        super(ImageTimelineRenderer, self).__init__(timeline, msg_combine_px=40.0)

        self.thumbnail_height = thumbnail_height

        self.thumbnail_combine_px = 20.0  # use cached thumbnail if it's less than this many pixels away
        self.min_thumbnail_width = 8  # don't display thumbnails if less than this many pixels across
        self.quality = Image.NEAREST  # quality hint for thumbnail scaling

        self.thumbnail_cache = TimelineCache(self._load_thumbnail, lambda topic, msg_stamp, thumbnail: self.timeline.scene().update())
    # TimelineRenderer implementation

    def get_segment_height(self, topic):
        return self.thumbnail_height

    def draw_timeline_segment(self, painter, topic, stamp_start, stamp_end, x, y, width, height):
        """
        draws a stream of images for the topic
        :param painter: painter object, ''QPainter''
        :param topic: topic to draw, ''str''
        :param stamp_start: stamp to start drawing, ''rospy.Time''
        :param stamp_end: stamp to end drawing, ''rospy.Time''
        :param x: x to draw images at, ''int''
        :param y: y to draw images at, ''int''
        :param width: width in pixels of the timeline area, ''int''
        :param height: height in pixels of the timeline area, ''int''
        """
        if x < self.timeline._history_left:
            width += x - self.timeline._history_left
            x = self.timeline._history_left
        max_interval_thumbnail = self.timeline.map_dx_to_dstamp(self.thumbnail_combine_px)
        max_interval_thumbnail = max(0.1, max_interval_thumbnail)
        thumbnail_gap = 6
        thumbnail_x, thumbnail_y, thumbnail_height = x + 1, y + 1, height - 2 - thumbnail_gap  # leave 1px border

        # set color to white draw rectangle over messages
        painter.setBrush(QBrush(Qt.white))
        painter.drawRect(x, y, width, height - thumbnail_gap)
        thumbnail_width = None

        while True:
            available_width = (x + width) - thumbnail_x

            # Check for enough remaining to draw thumbnail
            if width > 1 and available_width < self.min_thumbnail_width:
                break

            # Try to display the thumbnail, if its right edge is to the right of the timeline's left side
            if not thumbnail_width or thumbnail_x + thumbnail_width >= self.timeline._history_left:
                stamp = self.timeline.map_x_to_stamp(thumbnail_x, clamp_to_visible=False)
                thumbnail_bitmap = self.thumbnail_cache.get_item(topic, stamp, max_interval_thumbnail)

                # Cache miss
                if not thumbnail_bitmap:
                    thumbnail_details = (thumbnail_height,)
                    self.thumbnail_cache.enqueue((topic, stamp, max_interval_thumbnail, thumbnail_details))
                    if not thumbnail_width:
                        break
                else:
                    thumbnail_width, _ = thumbnail_bitmap.size

                    if width > 1:
                        if available_width < thumbnail_width:
                            thumbnail_width = available_width - 1
                    QtImage = ImageQt(thumbnail_bitmap)
                    pixmap = QPixmap.fromImage(QtImage)
                    painter.drawPixmap(thumbnail_x, thumbnail_y, thumbnail_width, thumbnail_height, pixmap)
            thumbnail_x += thumbnail_width

            if width == 1:
                break

        painter.setPen(QPen(QBrush(Qt.black)))
        painter.setBrush(QBrush(Qt.transparent))
        if width == 1:
            painter.drawRect(x, y, thumbnail_x - x, height - thumbnail_gap - 1)
        else:
            painter.drawRect(x, y, width, height - thumbnail_gap - 1)
        return True

    def close(self):
        if self.thumbnail_cache:
            self.thumbnail_cache.stop()
            self.thumbnail_cache.join()

    def _load_thumbnail(self, topic, stamp, thumbnail_details):
        """
        Loads the thumbnail from the bag
        """
        (thumbnail_height,) = thumbnail_details

        # Find position of stamp using index
        t = rospy.Time.from_sec(stamp)
        bag, entry = self.timeline.scene().get_entry(t, topic)
        if not entry:
            return None, None
        pos = entry.position

        # Not in the cache; load from the bag file

        with self.timeline.scene()._bag_lock:
            msg_topic, msg, msg_stamp = bag._read_message(pos)

        # Convert from ROS image to PIL image
        try:
            pil_image = image_helper.imgmsg_to_pil(msg)
        except Exception, ex:
            print >> sys.stderr, 'Error loading image on topic %s: %s' % (topic, str(ex))
            pil_image = None

        if not pil_image:
            print >> sys.stderr, 'Disabling renderer on %s' % topic
            self.timeline.set_renderer_active(topic, False)
            return None, None

        # Calculate width to maintain aspect ratio
        try:
            pil_image_size = pil_image.size
            thumbnail_width = int(round(thumbnail_height * (float(pil_image_size[0]) / pil_image_size[1])))
            # Scale to thumbnail size
            thumbnail = pil_image.resize((thumbnail_width, thumbnail_height), self.quality)

            return msg_stamp, thumbnail

        except Exception, ex:
            print >> sys.stderr, 'Error loading image on topic %s: %s' % (topic, str(ex))
            raise
            return None, None
