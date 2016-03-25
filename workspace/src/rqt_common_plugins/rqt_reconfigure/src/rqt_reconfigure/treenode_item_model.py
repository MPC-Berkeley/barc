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

from __future__ import division

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QStandardItemModel
import rospy


class TreenodeItemModel(QStandardItemModel):
    """
    This class is made only for this purpose; to hold QStandardItem instances
    associated with QModelIndex. QStandardItemModel has methods to return it
    by index called itemFromIndex, but in some cases the method doesn't work
    for unknown reasons. Ref. question asked:
    http://stackoverflow.com/questions/14646979/strange-index-values-from-qstandarditemmodel

    :author: Isaac Saito
    """

    def __init__(self, parent=None):
        super(TreenodeItemModel, self).__init__(parent)
        self._parent = parent

        self._indexes = {}  # { str : QPersistentModelIndex }

    def get_index_from_grn(self, grn):
        """

        :type grn: str
        :rtype: QPersistentModelIndex. None if the corresponding item isn't
                found.
        """
        rospy.logdebug('get_index_from_grn all item={}'.format(
                                                               self._indexes))
        return self._indexes.get(grn)

    def set_item_from_index(self, grn, qpindex):
        """
        :type grn: str
        :type qpindex: QPersistentModelIndex
        """
        rospy.logdebug('set_item_from_index grn={} qpindex={}'.format(
                                                               grn, qpindex))
        self._indexes[grn] = qpindex
