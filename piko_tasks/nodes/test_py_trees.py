#!/usr/bin/env python

"""
    test_py_trees.py - Version 1.0 2013-03-18
        
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2018 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import sys
import functools
import py_trees
import py_trees_ros
import py_trees.console as console
from py_trees.common import Status, BlackBoxLevel, VisibilityLevel
from py_trees.blackboard import Blackboard
import py_trees_msgs.msg as py_trees_msgs
from std_msgs.msg import Bool
from piko_tasks.servo_behaviors import CheckServosReady, InitServos, MoveHead, RelaxServos
from piko_tasks.move_base_behaviors import MoveBaseSimple

blackboard = Blackboard()

class PyTree():
    def __init__(self):
        rospy.init_node("test_py_trees")

        blackboard = Blackboard()
        
        root = self.create_root()
        behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
        
        py_trees.display.render_dot_tree(root, visibility_level=VisibilityLevel.BIG_PICTURE)
        
        rospy.on_shutdown(functools.partial(self.shutdown, behaviour_tree))
        
        if not behaviour_tree.setup(timeout=15):
            console.logerror("failed to setup the tree, aborting.")
            sys.exit(1)
        behaviour_tree.tick_tock(500)
        
    def create_root(self):
        root = py_trees.composites.Parallel("Root",)
        behave = py_trees.composites.Parallel("Behave")
        startup = py_trees.composites.Selector("Startup")
        check_active = py_trees.composites.Sequence("CheckActive")
        servo_tasks = py_trees.composites.Sequence("ServoTasks")
        move_base_tasks = py_trees.composites.Sequence("MoveBaseTasks")
        
        activate_to_bb = py_trees_ros.subscribers.ToBlackboard(
            name="Activate2BB",
            topic_name="/piko_dashboard/active",
            topic_type=Bool,
            blackboard_variables={'active': 'data'}
        )
        
        is_active = py_trees.blackboard.CheckBlackboardVariable(
            name="Active?",
            variable_name='active',
            expected_value=True
        )
        
        check_active.add_children([activate_to_bb, is_active])
        
        idle = py_trees.timers.Timer(name="Timer 3.0", duration=3.0)
        
        move_base_45     = MoveBaseSimple("Rotate 45 degrees", simple_goal=[0.0, 0.785398])
        move_base_neg_45 = MoveBaseSimple("Rotate -45 degrees", simple_goal=[0.0, -0.785398])

        move_base_tasks.add_children([move_base_45, idle, move_base_neg_45])
    
        check_servos_ready = CheckServosReady("CheckServosReady")
        init_servos        = InitServos("InitServos")
        
        startup.add_children([check_servos_ready, init_servos])

        center_head    = MoveHead("CenterHead", angle_goal=[0.0, 0.0])
        pan_head_right = MoveHead("PanHead",    angle_goal=[-1.0, 0.0])
        relax_servos   = RelaxServos("RelaxServos")
        
        servo_tasks.add_children([center_head, pan_head_right, relax_servos])

        behave.add_children([servo_tasks, move_base_tasks, idle])
        root.add_children([startup, check_active, behave])
        
        return root
            
    def shutdown(self, behaviour_tree):
        rospy.loginfo("Stopping servo task")
        for joint in sorted(blackboard.joints):
            print "Relax joint", joint
            blackboard.relax_servo[joint]()
            blackboard.enable_servo[joint](False)
            rospy.sleep(1.0)
        behaviour_tree.interrupt()

        rospy.sleep(1)

if __name__ == '__main__':
    tree = PyTree()

