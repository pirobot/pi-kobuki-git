#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Qt support for the tutorials.
"""

##############################################################################
# Imports
##############################################################################

import functools
import py_trees_ros
import rospy
import std_msgs.msg as std_msgs
import threading

from python_qt_binding.QtCore import Signal, Qt, QTimer, Slot
from python_qt_binding.QtWidgets import QWidget, QPushButton, QGridLayout, QSizePolicy, QLabel

##############################################################################
# Dashboard
##############################################################################


class Dashboard(QWidget):

    _activate_button_led = Signal(bool)
    _stop_button_led = Signal(bool)

    def __init__(self):
        super(Dashboard, self).__init__()

        not_latched = False
        # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            [
                ('activate', "~active", std_msgs.Bool, not_latched, 1),
                ('stop', "~active", std_msgs.Bool, not_latched, 1),
            ]
        )

        self.activate_push_button = QPushButton("Activate")
        self.activate_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")
        self.activate_push_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.activate_push_button.pressed.connect(functools.partial(self.publish_button_message, self.publishers.activate, True))

        self.stop_push_button = QPushButton("Stop")
        self.stop_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")
        self.stop_push_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.stop_push_button.pressed.connect(functools.partial(self.publish_button_message, self.publishers.stop, False))

        self.hbox_layout = QGridLayout(self)
        self.hbox_layout.addWidget(self.activate_push_button)
        self.hbox_layout.addWidget(self.stop_push_button)

        self.subscribers = py_trees_ros.utilities.Subscribers(
            [
                ("report", "/tree/report", std_msgs.String, self.reality_report_callback)
            ]
        )

    def publish_button_message(self, publisher, msg):
        publisher.publish(std_msgs.Bool(msg))

    def reality_report_callback(self, msg):
        if msg.data == False:
            self.set_activating_color(False)
            self.set_stopping_color(True)
            self.stop_push_button.setEnabled(True)
        elif msg.data == True:
            self.set_activating_color(True)
            self.set_stopping_color(False)
            self.stop_push_button.setEnabled(True)
        else:
            self.set_activating_color(True)
            self.set_stopping_color(False)

    def set_stopping_color(self, val):
        if val:
            self.stop_push_button.setStyleSheet("QPushButton { font-size: 30pt; background-color: red}")
        else:
            self.stop_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")

    def set_activating_color(self, val):
        if val:
            self.activate_push_button.setStyleSheet("QPushButton { font-size: 30pt; background-color: green}")
        else:
            self.activate_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")

