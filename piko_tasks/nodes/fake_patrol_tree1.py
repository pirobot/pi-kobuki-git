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
from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import Twist
from rbx2_msgs.srv import *
from pi_trees_lib3 import *
from pi_trees_ros3 import *
from rbx2_tasks.task_setup import *

# A class to track global variables
class BlackBoard():
    def __init__(self):
        # The robot's current position on the map
        self.robot_position = Point()
    
        self.robot_battery_level = None
        self.laptop_battery_level = None
        self.robot_charging = None
        self.laptop_charging = None
        self.manual_dock_in_progress = False
        self.off_docking_station = True # Fix for real robot - test if charging

# Initialize the black board
blackboard = BlackBoard()

class Patrol():
    def __init__(self):
        rospy.init_node("patrol_tree")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)

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
            
            MOVE_BASE_TASKS.append(move_base_task)
        
        # Set the docking station pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.docking_station_pose
        
        # Assign the docking station pose to a move_base action task
        NAV_DOCK_TASK = SimpleActionTask("NAV_DOC_TASK", "move_base", MoveBaseAction, goal, reset_after=True)
        
        # Create the root node
        BEHAVE = Sequence("BEHAVE")
        
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY")
        
        # The user can request a manual dock by publishing on the /manual_dock topic
        MANUAL_DOCK_REQUESTED = Selector("MANUAL_DOCK_REQUESTED")
        
        GET_OFF_DOCKING_STATION = Selector("GET_OFF_DOCKING_STATION")
        
        UNDOCK = CallbackTask("UNDOCK", cb=self.undock_cb)
        
        # Check to see if we're off the docking station
        CHECK_OFF_DOCKING_STATION = CallbackTask("CHECK_OFF_DOCKING_STATION", self.check_off_docking_station)

        GET_OFF_DOCKING_STATION.add_child(CHECK_OFF_DOCKING_STATION)
        GET_OFF_DOCKING_STATION.add_child(UNDOCK)
        
        # Create the patrol loop decorator
        LOOP_PATROL = Loop("LOOP_PATROL", iterations=self.n_patrols)
        
        # Create the patrol iterator
        PATROL = RandomIterator("PATROL")
        
        # Add the subtrees to the root node in order of priority
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(MANUAL_DOCK_REQUESTED)
        BEHAVE.add_child(GET_OFF_DOCKING_STATION)
        BEHAVE.add_child(LOOP_PATROL)
                
        # Add the move_base tasks to the patrol task
        for task in MOVE_BASE_TASKS:
            PATROL.add_child(task)
  
        # Add the patrol to the loop decorator
        LOOP_PATROL.add_child(PATROL)
        
        # The check battery condition (uses MonitorTask)
        CHECK_BATTERY = MonitorTask("CHECK_BATTERY", "battery_level", Float32, self.check_battery)
        
        # Allow requesting a manual dock by publishing an empty message on topic /manual_dock
        MONITOR_MANUAL_DOCK_REQUEST = MonitorTask("MONITOR_MANUAL_DOCK_REQUEST", "manual_dock", Empty, self.monitor_dock_request_cb, wait_for_message=False)
        
        # Check to see if a manual docking request is complete
        MANUAL_DOCK_COMPLETE = CallbackTask("MANUAL_DOCK_COMPLETE", self.manual_dock_complete_cb)
        
        CHECK_MANUAL_DOCK_COMPLETE = CallbackTask("CHECK_MANUAL_DOCK_COMPLETE", self.check_manual_dock_complete)

        # The charge robot task (uses ServiceTask)
        CHARGE_ROBOT = ServiceTask("CHARGE_ROBOT", "battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb=self.recharge_cb)
  
        # Build the recharge sequence using inline construction
        RECHARGE = Sequence("RECHARGE", [NAV_DOCK_TASK, CHARGE_ROBOT])
            
        MANUAL_DOCK_REQUESTED.add_child(CHECK_MANUAL_DOCK_COMPLETE)
        MANUAL_DOCK_REQUESTED.add_child(Sequence("MANUAL_DOCK", [NAV_DOCK_TASK, CHARGE_ROBOT, MANUAL_DOCK_COMPLETE]))
        
        # Add the check battery and recharge tasks to the stay healthy selector
        STAY_HEALTHY.add_child(CHECK_BATTERY)
        STAY_HEALTHY.add_child(RECHARGE)
                
        # Display the tree before beginning execution
        print "Patrol Behavior Tree"
        print_tree(BEHAVE, use_symbols=True)
        
        print_dot_tree(BEHAVE)
        
        #print_phpsyntax_tree(BEHAVE)
        
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(1)
            
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
        blackboard.manual_dock_in_progress = False
            
    def monitor_dock_request_cb(self, msg):
        rospy.loginfo("MANUAL DOCK REQUESTED!")
        blackboard.manual_dock_in_progress = True
        
        return TaskStatus.SUCCESS
    
    def check_manual_dock_complete(self):
        if blackboard.manual_dock_in_progress:
            return False
        else:
            return True
        
    def manual_dock_complete_cb(self):
        rospy.loginfo("MANUAL DOCK COMPLETE")
        blackboard.manual_dock_in_progress = False
        blackboard.off_docking_station = False

    def check_off_docking_station(self):
        if blackboard.off_docking_station:
            return True
        else:
            return False
        
    def undock_cb(self):
        rospy.loginfo("LEAVING THE DOCKING STATION")

        cmd_vel = Twist()
        
        cmd_vel.linear.x = -0.05
        
        timer = 0
        
        while timer < 5:
            self.cmd_vel_pub.publish(cmd_vel)
            timer += 0.1
            rospy.sleep(0.1)
            
        blackboard.off_docking_station = True
        
        return True
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    tree = Patrol()

