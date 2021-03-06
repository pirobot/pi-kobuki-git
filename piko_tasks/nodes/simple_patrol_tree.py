#!/usr/bin/env python

"""
    patrol_tree.py - Version 1.0 2013-03-18
    
    Navigate a series of waypoints while monitoring battery levels.
    Uses the pi_trees package to implement a behavior tree task manager.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

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
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rbx2_msgs.srv import *
from pi_trees_ros.pi_trees_ros import *
from piko_tasks.task_setup import *

class Patrol():
    def __init__(self):
        rospy.init_node("patrol_tree")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Create the root node
        BEHAVE = Sequence("BEHAVE")

        # Create a list to hold the move_base tasks
        MOVE_BASE_TASKS = list()
        
        n_waypoints = len(self.waypoints)
        
        # Create simple action navigation task for each waypoint
        for i in range(n_waypoints + 1):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.waypoints[i % n_waypoints]
            
            move_base_task = SimpleActionTask("MOVE_BASE_TASK_" + str(i), "move_base", MoveBaseAction, goal, reset_after=False)
            until_success_move_base_task = UntilSuccess("UNTIL_SUCCESS_MOVE_BASE_TASK_" + str(i), max_attempts=3, root=BEHAVE)
            until_success_move_base_task.add_child(move_base_task)
            
            MOVE_BASE_TASKS.append(until_success_move_base_task)
        
        # Set the docking station pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.docking_station_pose
        
        # Assign the docking station pose to a move_base action task
        NAV_DOCK_TASK = SimpleActionTask("NAV_DOC_TASK", "move_base", MoveBaseAction, goal, reset_after=False)

        # Create the "stay healthy" selector
        #STAY_HEALTHY = Selector("STAY_HEALTHY")
        
        # Create the patrol loop decorator
        LOOP_PATROL = Loop("LOOP_PATROL", iterations=self.n_patrols, root=BEHAVE)
        
        # Add the two subtrees to the root node in order of priority
        #BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(LOOP_PATROL)
        
        # Create the patrol iterator
        PATROL = Iterator("PATROL")
        
        # Add the move_base tasks to the patrol task
        for task in MOVE_BASE_TASKS:
            PATROL.add_child(task)
  
        # Add the patrol to the loop decorator
        LOOP_PATROL.add_child(PATROL)
                
        # Display the tree before beginning execution
        print "Patrol Behavior Tree"
        print_tree(BEHAVE, use_symbols=True)
        print_dot_tree(BEHAVE)
        
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.2)
            print_dot_tree(BEHAVE)

            
    def check_battery(self, msg):
        if msg.data is None:
            return TaskStatus.RUNNING
        else:
            if msg.data < self.low_battery_threshold:
                rospy.loginfo("LOW BATTERY - level: " + str(int(msg.data)))
                return TaskStatus.FAILURE
            else:
                return TaskStatus.SUCCESS
    
    def recharge_cb(self, result):
        rospy.loginfo("BATTERY CHARGED!")
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    tree = Patrol()

