#! /usr/bin/env python

import rospy


# Provides callback functions for the start and stop buttons
class NodeController(object):
    '''
    Containing both proxy and gui instances, this class gives a control of
    a node on both ROS & GUI sides.
    '''

    # these values need to synch with member variables.
    # Eg. self.gui isn't legal.
    __slots__ = ['_proxy', '_gui']

    def __init__(self, proxy, gui):
        '''
        @type proxy: rqt_launch.NodeProxy
        @type gui: QWidget
        '''
        self._proxy = proxy

        self._gui = gui
        self._gui.set_node_controller(self)

    def start_stop_slot(self, signal):
        '''
        Works as a slot particularly intended to work for
        QAbstractButton::toggled(checked). Internally calls
        NodeController.start / stop depending on `signal`.

        @type signal: bool
        '''
        if self._proxy.is_running():
            self.stop()
            rospy.logdebug('---start_stop_slot stOP')
        else:
            self.start()
            rospy.logdebug('==start_stop_slot StART')

    def start(self, restart=True):
        '''
        Start a ROS node as a new _process.
        '''
        rospy.logdebug('Controller.start restart={}'.format(restart))

        # Should be almost unreachable under current design where this 'start'
        # method only gets called when _proxy.is_running() returns false.

        if self._proxy.is_running():
            if not restart:
                # If the node is already running and restart isn't desired,
                # do nothing further.
                return
            #TODO: Need to consider...is stopping node here
            # (i.e. in 'start' method) good?
            self.stop()

        # If the launch_prefix has changed, then the _process must be recreated
        if (self._proxy.config.launch_prefix !=
            self._gui._lineEdit_launch_args.text()):

            self._proxy.config.launch_prefix = \
                                     self._gui._lineEdit_launch_args.text()
            self._proxy.recreate_process()

        self._gui.set_node_started(False)
        self._gui.label_status.set_starting()
        self._proxy.start_process()
        self._gui.label_status.set_running()
        self._gui.label_spawncount.setText("({})".format(
                                              self._proxy.get_spawn_count()))

    def stop(self):
        '''
        Stop a ROS node's _process.
        '''

        #TODO: Need to check if the node is really running.

        #if self._proxy.is_running():
        self._gui.set_node_started(True)
        self._gui.label_status.set_stopping()
        self._proxy.stop_process()
        self._gui.label_status.set_stopped()

    def check_process_status(self):
        if self._proxy.has_died():
            rospy.logerr("Process died: {}".format(
                                                  self._proxy.get_proc_name()))
            self._proxy.stop_process()
            self._gui.set_node_started(True)
            if self._proxy._process.exit_code == 0:
                self._gui.label_status.set_stopped()
            else:
                self._gui.label_status.set_died()

            # Checks if it should be respawned
            if self._gui.respawn_toggle.isChecked():
                rospy.loginfo("Respawning _process: {}".format(
                                                    self._proxy._process.name))
                self._gui.label_status.set_starting()
                self._proxy.start_process()
                self._gui.label_status.set_running()
                self._gui.label_spawncount.setText("({})".format(
                                             self._proxy._process.spawn_count))

    def get_node_widget(self):
        '''
        @rtype: QWidget
        '''
        return self._gui

    def is_node_running(self):
        return self._proxy.is_running()
