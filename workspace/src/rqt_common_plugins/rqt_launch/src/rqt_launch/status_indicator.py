#! /usr/bin/env python

from python_qt_binding.QtGui import QLabel, QStyle
import rospy


class StatusIndicator(QLabel):
    def __init__(self, *args):
        super(StatusIndicator, self).__init__(*args)
        self.set_stopped()

    def set_running(self):
        self.setPixmap(
           self.style().standardIcon(QStyle.SP_DialogApplyButton).pixmap(16))

    def set_starting(self):
        rospy.logdebug('StatusIndicator.set_starting')
        self.setPixmap(self.style().standardIcon(
                                      QStyle.SP_DialogResetButton).pixmap(16))

    def set_stopping(self):
        """
        Show msg that the process is "stopping".

        cf. set_stopped()
        """
        self.setPixmap(self.style().standardIcon(
                                      QStyle.SP_DialogResetButton).pixmap(16))

    def set_stopped(self):
        """
        Show msg that the process is "stopped".

        cf. set_stopping()
        """
        self.setText(" ")

    def set_died(self):
        self.setPixmap(self.style().standardIcon(
                                      QStyle.SP_MessageBoxCritical).pixmap(16))
