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


from python_qt_binding.QtGui import QModelIndex


class TreenodeStatus(QModelIndex):
    """

    This class contains very similar information with
    rqt_reconfigure.ParameterItem. The purpose of this class is to enable
    FilterChildrenModel (subclassing QSortFilterProxyModel) to look up each
    node, which, afaik, is not possible via QSortFilterProxyModel and that's
    why I created this class.

    That said, to store an info about each treenode:

    - ParameterItem should be used to show on view.
    - This class should be used when you need to keep track from
      QAbstractProxyModel

    :author: Isaac Saito
    """

    def __init__(self, nodename_full=None, qmindex=None):
        """
        :param index_id: default value is -1, which indicates "not set". This
                         can be set.
        :param nodename_full: default value is None, which indicates "not set".
                        This can be set.
        :type index_id: qint64
        :type nodename_full: str
        :type qmindex: QModelIndex
        """
        super(TreenodeStatus, self).__init__(qmindex)

        self._is_eval_done = False
        self._shows = False
        self._nodename_full = nodename_full

    def set_nodename_full(self, nodename_full):
        self._nodename_full = nodename_full

    def get_nodename_full(self):
        return self._nodename_full

    def set_is_eval_done(self, v):
        self._is_eval_done = v

    def get_is_eval_done(self):
        return self._is_eval_done

    def set_shows(self, v):
        self._shows = v

    def get_shows(self):
        return self._shows
