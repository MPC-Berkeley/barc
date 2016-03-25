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

import os
import sys

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QModelIndex, Signal
from python_qt_binding.QtGui import QDialog, QStandardItem, \
                                    QStandardItemModel
from rosgraph import rosenv
import roslaunch
from roslaunch.core import RLException
import rospkg
import rospy

#from rqt_console.console_widget import ConsoleWidget
from rqt_launch.node_proxy import NodeProxy
from rqt_launch.node_controller import NodeController
from rqt_launch.node_delegate import NodeDelegate
from rqt_launch.status_indicator import StatusIndicator
from rqt_py_common.rqt_roscomm_util import RqtRoscommUtil


class LaunchWidget(QDialog):
    '''#TODO: comment
    '''

    # To be connected to PluginContainerWidget
    sig_sysmsg = Signal(str)

    def __init__(self, parent):
        '''
        @type parent: LaunchMain
        '''
        super(LaunchWidget, self).__init__()
        self._parent = parent

        self._config = None

        #TODO: should be configurable from gui
        self._port_roscore = 11311

        self._run_id = None

        self._rospack = rospkg.RosPack()
        ui_file = os.path.join(self._rospack.get_path('rqt_launch'),
                               'resource', 'launch_widget.ui')
        loadUi(ui_file, self)

        # row=0 allows any number of rows to be added.
        self._datamodel = QStandardItemModel(0, 1)

        master_uri = rosenv.get_master_uri()
        rospy.loginfo('LaunchWidget master_uri={}'.format(master_uri))
        self._delegate = NodeDelegate(master_uri,
                                      self._rospack)
        self._treeview.setModel(self._datamodel)
        self._treeview.setItemDelegate(self._delegate)

        # NodeController used in controller class for launch operation.
        self._node_controllers = []

        self._pushbutton_start_all.clicked.connect(self._parent.start_all)
        self._pushbutton_stop_all.clicked.connect(self._parent.stop_all)
        # Bind package selection with .launch file selection.
        self._combobox_pkg.currentIndexChanged[str].connect(
                                                 self._refresh_launchfiles)
        # Bind a launch file selection with launch GUI generation.
        self._combobox_launchfile_name.currentIndexChanged[str].connect(
                                                 self._load_launchfile_slot)
        self._update_pkgs_contain_launchfiles()

        # Used for removing previous nodes
        self._num_nodes_previous = 0

    def _load_launchfile_slot(self, launchfile_name):
        # Not yet sure why, but everytime combobox.currentIndexChanged occurs,
        # this method gets called twice with launchfile_name=='' in 1st call.
        if launchfile_name == None or launchfile_name == '':
            return

        _config = None

        rospy.logdebug('_load_launchfile_slot launchfile_name={}'.format(
                                                           launchfile_name))

        try:
            _config = self._create_launchconfig(launchfile_name,
                                                self._port_roscore)
            #TODO: folder_name_launchfile should be able to specify arbitrarily
            # _create_launchconfig takes 3rd arg for it.

        except IndexError as e:
            msg = 'IndexError={} launchfile={}'.format(e.message,
                                                       launchfile_name)
            rospy.logerr(msg)
            self.sig_sysmsg.emit(msg)
            return
        except RLException as e:
            msg = 'RLException={} launchfile={}'.format(e.message,
                                                        launchfile_name)
            rospy.logerr(msg)
            self.sig_sysmsg.emit(msg)
            return

        self._create_widgets_for_launchfile(_config)

    def _create_launchconfig(self, launchfile_name, port_roscore=11311,
                             folder_name_launchfile='launch'):
        '''
        @rtype: ROSLaunchConfig
        @raises RLException: raised by roslaunch.config.load_config_default.
        '''

        pkg_name = self._combobox_pkg.currentText()

        try:
            launchfile = os.path.join(self._rospack.get_path(pkg_name),
                                      folder_name_launchfile, launchfile_name)
        except IndexError as e:
            raise RLException('IndexError: {}'.format(e.message))

        try:
            launch_config = roslaunch.config.load_config_default([launchfile],
                                                                 port_roscore)
        except rospkg.common.ResourceNotFound as e:
            raise RLException('ResourceNotFound: {}'.format(e.message))
        except RLException as e:
            raise e

        return launch_config

    def _create_widgets_for_launchfile(self, config):
        self._config = config

        # Delete old nodes' GUIs.
        self._node_controllers = []

        # These lines seem to remove indexWidgets previously set on treeview.
        # Per suggestion in API doc, we are not using reset(). Instead,
        # using 2 methods below without any operation in between, which
        # I suspect might be inproper.
        # http://qt-project.org/doc/qt-4.8/qabstractitemmodel.html#reset
        self._datamodel.beginResetModel()
        self._datamodel.endResetModel()

        # Need to store the num of nodes outside of the loop -- this will
        # be used for removing excessive previous node rows.
        order_xmlelement = 0
        # Loop per xml element
        for order_xmlelement, node in enumerate(self._config.nodes):
            proxy = NodeProxy(self._run_id, self._config.master.uri, node)

            # TODO: consider using QIcon.fromTheme()
            status_label = StatusIndicator()

            qindex_nodewidget = self._datamodel.index(order_xmlelement,
                                                       0, QModelIndex())
            node_widget = self._delegate.create_node_widget(qindex_nodewidget,
                                                            proxy.config,
                                                            status_label)

            #TODO: Ideally find a way so that we don't need this block.
            #BEGIN If these lines are missing, widget won't be shown either.
            std_item = QStandardItem(
                                     #node_widget.get_node_name()
                                     )
            self._datamodel.setItem(order_xmlelement, 0, std_item)
            #END If these lines are missing, widget won't be shown either.

            self._treeview.setIndexWidget(qindex_nodewidget, node_widget)

            node_controller = NodeController(proxy, node_widget)
            self._node_controllers.append(node_controller)

            node_widget.connect_start_stop_button( \
                                       node_controller.start_stop_slot)
            rospy.logdebug('loop #%d proxy.config.namespace=%s ' +
                          'proxy.config.name=%s',
                          order_xmlelement, proxy.config.namespace,
                          proxy.config.name)

        self._num_nodes_previous = order_xmlelement

        self._parent.set_node_controllers(self._node_controllers)
        self._parent.set_config(config)

    def _update_pkgs_contain_launchfiles(self):
        '''
        Inspired by rqt_msg.MessageWidget._update_pkgs_contain_launchfiles
        '''
        packages = sorted([pkg_tuple[0]
                           for pkg_tuple
                           in RqtRoscommUtil.iterate_packages('launch')])
        self._package_list = packages
        rospy.logdebug('pkgs={}'.format(self._package_list))
        self._combobox_pkg.clear()
        self._combobox_pkg.addItems(self._package_list)
        self._combobox_pkg.setCurrentIndex(0)

    def _refresh_launchfiles(self, package=None):
        '''
        Inspired by rqt_msg.MessageWidget._refresh_msgs
        '''
        if package is None or len(package) == 0:
            return
        self._launchfile_instances = []  # Launch[]
        #TODO: RqtRoscommUtil.list_files's 2nd arg 'subdir' should NOT be
        # hardcoded later.
        _launch_instance_list = RqtRoscommUtil.list_files(package,
                                                         'launch')

        rospy.logdebug('_refresh_launches package={} instance_list={}'.format(
                                                       package,
                                                       _launch_instance_list))

        self._launchfile_instances = [x.split('/')[1]
                                      for x in _launch_instance_list]

        self._combobox_launchfile_name.clear()
        self._combobox_launchfile_name.addItems(self._launchfile_instances)

    def save_settings(self, plugin_settings, instance_settings):
        # instance_settings.set_value('splitter', self._splitter.saveState())
        pass

    def restore_settings(self, plugin_settings, instance_settings):
#        if instance_settings.contains('splitter'):
#            self._splitter.restoreState(instance_settings.value('splitter'))
#        else:
#            self._splitter.setSizes([100, 100, 200])
        pass


if __name__ == '__main__':
    # main should be used only for debug purpose.
    # This launches this QWidget as a standalone rqt gui.
    from rqt_gui.main import Main

    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_launch'))
