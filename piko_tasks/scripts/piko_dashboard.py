#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_ros/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
Launch a qt dashboard for the Pi Kobuki.
"""
##############################################################################
# Imports
##############################################################################

from piko_tasks.qt import Dashboard
import rospy
import signal
import sys

from python_qt_binding.QtWidgets import QApplication, QMainWindow

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('piko_dashboard')
    app = QApplication(sys.argv)
    window = QMainWindow()
    dashboard = Dashboard()
    window.setCentralWidget(dashboard)
    window.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
