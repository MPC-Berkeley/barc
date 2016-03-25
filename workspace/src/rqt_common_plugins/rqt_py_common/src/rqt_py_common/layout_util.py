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
#
# Author: Isaac Saito

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QWidgetItem
import roslib
import rospy


class LayoutUtil(object):

    @staticmethod
    def alternate_color(list_widgets, colors_alter=[Qt.white, Qt.gray]):
        """
        Alternate the background color of the widgets that are ordered
        linearly, by the given list of colors.

        Originally intended for the elements of QHBoxLayout & QVBoxLayout.

        @type list_widgets: QtGui.QWidget[]
        @type colors_alter: QtCore.Qt.GlobalColor[]
        @param colors_alter: 1st element is used as initial/default color.
        @rtype: void

        @author: Isaac Saito
        """

        colors_num = len(colors_alter)
        i_widget = 0
        for w in list_widgets:
            w.setAutoFillBackground(True)
            p = w.palette()

            divisor = (i_widget + colors_num) % colors_num
            i_widget += 1

            rospy.logdebug('LayoutUtil divisor={} i_widget={} colors_num={}'.format(
                                                                   divisor,
                                                                   i_widget,
                                                                   colors_num))

            p.setColor(w.backgroundRole(), colors_alter[divisor])
            w.setPalette(p)

    @staticmethod
    def clear_layout(layout):
        """
        Clear all items in the given layout. Currently, only the instances of
        QWidgetItem get cleared (ie. QSpaceItem is ignored).

        Originally taken from http://stackoverflow.com/a/9375273/577001

        :type layout: QLayout
        """
        for i in reversed(range(layout.count())):
            item = layout.itemAt(i)

            if isinstance(item, QWidgetItem):
                # print "widget" + str(item)
                item.widget().close()
                # or
                # item.widget().setParent(None)
            elif isinstance(item, QSpacerItem):
                # print "spacer " + str(item)
                continue
                # no need to do extra stuff
            else:
                # print "layout " + str(item)
                LayoutUtil.clear_layout(item.layout())

            # remove the item from layout
            layout.removeItem(item)
