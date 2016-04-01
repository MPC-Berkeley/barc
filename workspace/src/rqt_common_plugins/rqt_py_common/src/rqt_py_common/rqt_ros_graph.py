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
import rospy


class RqtRosGraph(object):

    DELIM_GRN = '/'

    @staticmethod
    def get_full_grn(model_index):
        """
        @deprecated: Not completed.

        Create full path format of GRN (Graph Resource Names, see
        http://www.ros.org/wiki/Names). Build GRN by recursively transcending
        parents & children of a given QModelIndex instance.

        A complete example of GRN: /wide_stereo/left/image_color/compressed

        Upon its very 1st call, the argument is the index where user clicks on
        on the view object (here QTreeView is used but should work with other
        View too. Not tested yet though). str_grn can be 0-length string.

        :type model_index: QModelIndex
        :type str_grn: str
        :param str_grn: This could be an incomplete or a complete GRN format.
        :rtype: str
        """

        children_grn_list = RqtRosGraph.get_lower_grn_dfs(model_index)
        parent_data = model_index.data()
        rospy.logdebug('parent_data={}'.format(parent_data))
        if parent_data == None:  # model_index is 1st-order node of a tree.
            upper_grn = RqtRosGraph.get_upper_grn(model_index, '')
            grn_list = []
            for child_grn in children_grn_list:
                grn_full = upper_grn + child_grn
                rospy.logdebug('grn_full={} upper_grn={} child_grn={}'.format(
                                               grn_full, upper_grn, child_grn))
                grn_list.append(grn_full)
        else:
            grn_list = children_grn_list

        #Create a string where namespace is delimited by slash.
        grn = ''
        for s in grn_list:
            grn += RqtRosGraph.DELIM_GRN + s

        return grn

    @staticmethod
    def get_lower_grn_dfs(model_index, grn_prev=''):
        """
        Traverse all children treenodes and returns a list of "partial"
        GRNs. Partial means that this method returns names under current level.

        Ex. Consider a tree like this:

        Root
         |--TopitemA
         |    |--1
         |      |--2
         |        |--3
         |          |--4
         |          |--5
         |            |--6
         |            |--7
         |--TopitemB

        Re-formatted in GRN (omitting root):

          TopitemA/1/2/3/4
          TopitemA/1/2/3/5/6
          TopitemA/1/2/3/5/7
          TopitemB

         Might not be obvious from tree representation but there are 4 nodes as
         GRN form suggests.

         (doc from here TBD)

        :type model_index: QModelIndex
        :type grn_prev: str
        :rtype: str[]
        """
        i_child = 0
        list_grn_children_all = []
        while True:  # Loop per child.
            grn_curr = grn_prev + RqtRosGraph.DELIM_GRN + str(
                                                            model_index.data())
            child_qmindex = model_index.child(i_child, 0)

            if (not child_qmindex.isValid()):
                rospy.logdebug('!! DEADEND i_child=#{} grn_curr={}'.format(
                                                           i_child, grn_curr))
                if i_child == 0:
                    # Only when the current node has no children, add current
                    # GRN to the returning list.
                    list_grn_children_all.append(grn_curr)
                return list_grn_children_all

            rospy.logdebug('Child#{} grn_curr={}'.format(i_child, grn_curr))

            list_grn_children = RqtRosGraph.get_lower_grn_dfs(child_qmindex,
                                                              grn_curr)
            for child_grn in list_grn_children:
                child_grn = (grn_prev +
                             (RqtRosGraph.DELIM_GRN + grn_curr) +
                             (RqtRosGraph.DELIM_GRN + child_grn))

            list_grn_children_all = list_grn_children_all + list_grn_children
            rospy.logdebug('111 lennodes={} list_grn_children={}'.format(
                                len(list_grn_children_all), list_grn_children))
            rospy.logdebug('122 list_grn_children_all={}'.format(
                                                        list_grn_children_all))
            i_child += 1
        return list_grn_children_all

    @staticmethod
    def get_upper_grn(model_index, str_grn):
        if model_index.data(Qt.DisplayRole) == None:
            return str_grn
        str_grn = (RqtRosGraph.DELIM_GRN +
                   str(model_index.data(Qt.DisplayRole)) +
                   str_grn)
        rospy.logdebug('get_full_grn_recur out str=%s', str_grn)
        return RqtRosGraph.get_upper_grn(model_index.parent(), str_grn)
