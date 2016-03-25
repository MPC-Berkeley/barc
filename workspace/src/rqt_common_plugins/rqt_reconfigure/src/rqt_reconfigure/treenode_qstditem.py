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

import threading
import time

import dynamic_reconfigure.client
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QBrush, QStandardItem
import rospy
from rospy.exceptions import ROSException
from rqt_py_common.data_items import ReadonlyItem

from .dynreconf_client_widget import DynreconfClientWidget


class ParamserverConnectThread(threading.Thread):
    def __init__(self, parent, param_name_raw):
        super(ParamserverConnectThread, self).__init__()
        self._parent = parent
        self._param_name_raw = param_name_raw

    def run(self):
        dynreconf_client = None
        try:
            dynreconf_client = dynamic_reconfigure.client.Client(
                                       str(self._param_name_raw), timeout=5.0)
            rospy.logdebug('ParamserverConnectThread dynreconf_client={}'. \
                          format(dynreconf_client))
            self._parent.set_dynreconf_client(dynreconf_client)
        except rospy.exceptions.ROSException as e:
            raise type(e)(e.message +
                          "TreenodeQstdItem. Couldn't connect to {}".format(
                                                         self._param_name_raw))


class TreenodeQstdItem(ReadonlyItem):
    """
    Extending ReadonlyItem - the display content of this item shouldn't be
    modified.
    """

    NODE_FULLPATH = 1

    def __init__(self, *args):
        """
        :param args[0]: str (will become 1st arg of QStandardItem)
        :param args[1]: integer value that indicates whether this class
                               is node that has GRN (Graph Resource Names, see
                               http://www.ros.org/wiki/Names). This can be None
        """
        grn_current_treenode = args[0]
        self._param_name_raw = grn_current_treenode
        self._set_param_name(grn_current_treenode)
        super(TreenodeQstdItem, self).__init__(grn_current_treenode)

        # dynamic_reconfigure.client.Client
        self._dynreconf_client = None
        # DynreconfClientWidget
        self._dynreconfclient_widget = None

        self._is_rosnode = False

        self._lock = None
        self._paramserver_connect_thread = None

        try:
            if args[1]:
                self._is_rosnode = True
        except IndexError:  # tuple index out of range etc.
                rospy.logerr('TreenodeQstdItem IndexError')

    def set_dynreconf_client(self, dynreconf_client):
        """
        @param dynreconf_client: dynamic_reconfigure.client.Client
        """
        self._dynreconf_client = dynreconf_client
        rospy.logdebug('Qitem set dynreconf_client={} param={}'.format(
                                                       self._dynreconf_client,
                                                       self._param_name_raw))

    def clear_dynreconf_client(self):
        if self._dynreconf_client is not None:
            self._dynreconf_client.close()
            del self._dynreconf_client
            self._dynreconf_client = None

    def get_dynreconf_widget(self):
        """
        @rtype: DynreconfClientWidget (QWidget)
        @return: None if dynreconf_client is not yet generated.
        @raise ROSException:
        """

        if not self._dynreconfclient_widget:
            rospy.logdebug('get dynreconf_client={}'.format(
                                                       self._dynreconf_client))
            rospy.logdebug('In get_dynreconf_widget 1')
            if not self._dynreconf_client:
                self.connect_param_server()
            rospy.logdebug('In get_dynreconf_widget 2')

            timeout = 3 * 100
            loop = 0
            # Loop until _dynreconf_client is set. self._dynreconf_client gets
            # set from different thread (in ParamserverConnectThread).
            while self._dynreconf_client == None:
                #Avoid deadlock
                if timeout < loop:
                    #Make itself unclickable
                    self.setEnabled(False)
                    raise ROSException('dynreconf client failed')

                time.sleep(0.01)
                loop += 1
                rospy.logdebug('In get_dynreconf_widget loop#{}'.format(loop))

            rospy.logdebug('In get_dynreconf_widget 4')
            self._dynreconfclient_widget = DynreconfClientWidget(
                                                       self._dynreconf_client,
                                                       self._param_name_raw)
            # Creating the DynreconfClientWidget transfers ownership of the _dynreconf_client
            # to it. If it is destroyed from Qt, we need to clear our reference to it and
            # stop the param server thread we had.
            self._dynreconfclient_widget.destroyed.connect(self.clear_dynreconfclient_widget)
            self._dynreconfclient_widget.destroyed.connect(self.disconnect_param_server)
            rospy.logdebug('In get_dynreconf_widget 5')

        else:
            pass
        return self._dynreconfclient_widget

    def clear_dynreconfclient_widget(self):
        self._dynreconfclient_widget = None

    def connect_param_server(self):
        """
        Connect to parameter server using dynamic_reconfigure client.
        Behavior is delegated to a private method _connect_param_server, and
        its return value, client, is set to member variable.

        @return void
        @raise ROSException:
        """
        # If the treenode doesn't represent ROS Node, return None.
        if not self._is_rosnode:
            rospy.logerr('connect_param_server failed due to missing ' +
                         'ROS Node. Return with nothing.')
            return

        if not self._dynreconf_client:
            if self._paramserver_connect_thread:
                self.disconnect_param_server()
            self._paramserver_connect_thread = ParamserverConnectThread(
                                       self, self._param_name_raw)
            self._paramserver_connect_thread.start()

    def disconnect_param_server(self):
        if self._paramserver_connect_thread:
            # Try to stop the thread
            if self._paramserver_connect_thread.isAlive():
              self._paramserver_connect_thread.join(1)
            del self._paramserver_connect_thread
            self._paramserver_connect_thread = None
        self.clear_dynreconf_client()

    def enable_param_items(self):
        """
        Create QStdItem per parameter and addColumn them to myself.
        :rtype: None if _dynreconf_client is not initiated.
        """
        if not self._dynreconfclient_widget:
            return None
        paramnames = self._dynreconfclient_widget.get_treenode_names()
        paramnames_items = []
        brush = QBrush(Qt.lightGray)
        for paramname in paramnames:
            item = ReadonlyItem(paramname)
            item.setBackground(brush)
            paramnames_items.append(item)
        rospy.logdebug('enable_param_items len of paramnames={}'.format(
                                                        len(paramnames_items)))
        self.appendColumn(paramnames_items)

    def _set_param_name(self, param_name):
        """
        :param param_name: A string formatted as GRN (Graph Resource Names, see
                           http://www.ros.org/wiki/Names).
                           Example: /paramname/subpara/subsubpara/...
        """
        rospy.logdebug('_set_param_name param_name={} '.format(param_name))

        #  separate param_name by forward slash
        self._list_treenode_names = param_name.split('/')

        #  Deleting the 1st elem which is zero-length str.
        del self._list_treenode_names[0]

        self._toplevel_treenode_name = self._list_treenode_names[0]

        rospy.logdebug('paramname={} nodename={} _list_params[-1]={}'.format(
                       param_name, self._toplevel_treenode_name,
                       self._list_treenode_names[-1]))

    def get_param_name_toplv(self):
        """
        :rtype: String of the top level param name.
        """

        return self._name_top

    def get_raw_param_name(self):
        return self._param_name_raw

    def get_treenode_names(self):
        """
        :rtype: List of string. Null if param
        """

        #TODO: what if self._list_treenode_names is empty or null?
        return self._list_treenode_names

    def get_node_name(self):
        """
        :return: A value of single tree node (ie. NOT the fullpath node name).
                 Ex. suppose fullpath name is /top/sub/subsub/subsubsub and you
                     are at 2nd from top, the return value is subsub.
        """
        return self._toplevel_treenode_name

    def type(self):
        return QStandardItem.UserType
