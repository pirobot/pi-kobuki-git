#!/usr/bin/env python

"""
    patrol_tree.py - Version 1.0 2016-01-18
    
    Navigate a series of waypoints while monitoring battery levels.
    Uses the pi_trees package to implement a behavior tree task manager.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2016 Patrick Goebel.  All rights reserved.

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
from pi_trees_ros3.pi_trees_ros3 import *
from pi_trees_lib3 import *
from piko_msgs.msg import Dock
from piko_tasks.task_setup import *
import thread
import time
import os
import copy

# Autodocking stuff
import actionlib
from actionlib_msgs.msg import GoalStatus

class Patrol():
    def __init__(self):
        rospy.init_node("patrol_tree", log_level=rospy.DEBUG)
        #rospy.init_node("patrol_tree")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Are we running the fake robot?
        self.fake = rospy.get_param('~fake', False)
        
        # How frequently do we "tic" the tree?
        rate = rospy.get_param('~rate', 10)
        
        # Convert tick rate to a ROS rate
        tick_rate = rospy.Rate(rate)
        
        # Where should the DOT file be stored.  Default location is $HOME/.ros/tree.dot
        dotfilepath = rospy.get_param('~dotfilepath', None)

        # What do we consider a full charge?
        self.full_charge = rospy.get_param('~full_charge', 100)
        
        # Initialize the blackboard
        self.blackboard = Blackboard("blackboard", debug=True)
        self.blackboard.fake = self.fake
        self.init_blackboard(self.blackboard)
        
        # The root node
        BEHAVE = Sequence("BEHAVE")
        
        # The behavior tree
        self.tree = BehaviorTree("Patrol Tree", root=BEHAVE, blackboard=self.blackboard)
        
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY", tree=self.tree, announce=True)
        
        # Check for manual requests
        CHECK_USER_REQUESTS = Selector("CHECK_USER_REQUESTS", tree=self.tree, announce=True)
        
        # Create the patrol iterator
        PATROL = Iterator("PATROL", tree=self.tree, reset_after=False, announce=True)
        
        # Create a list to hold the move_base tasks
        MOVE_BASE_TASKS = list()
        
        # The self.waypoints list is defined in the task_setup.py file imported at the top of this file
        n_waypoints = len(self.waypoints)
        
        # Create a simple action navigation task for each waypoint
        for i in range(n_waypoints):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.waypoints[i]
            
            move_base_task = SimpleActionTask("MOVE_BASE_TASK_" + str(i), "move_base", MoveBaseAction, goal, tree=self.tree, connect_timeout=1, result_timeout=300, reset_after=False)
            
            MOVE_BASE_TASKS.append(move_base_task)
        
        # Set the pre-docking station pose.  The pose is defined in task_setup.py.
        near_dock_goal = MoveBaseGoal()
        near_dock_goal.target_pose.header.frame_id = 'map'
        near_dock_goal.target_pose.header.stamp = rospy.Time.now()
        near_dock_goal.target_pose.pose = self.pre_dock_pose
        
        # Assign the near docking station pose to a move_base action task
        NAV_NEAR_DOCK = SimpleActionTask("NAV_NEAR_DOCK", "move_base", MoveBaseAction, near_dock_goal, tree=self.tree, reset_after=False)
        
        # Set the pose of the docking station itself. The pose is defined in task_setup.py.
        dock_goal = MoveBaseGoal()
        dock_goal.target_pose.header.frame_id = 'map'
        dock_goal.target_pose.header.stamp = rospy.Time.now()
        dock_goal.target_pose.pose = self.docking_station_pose
                
        # Get the fake diagnostics like battery charge.
        GET_DIAGNOSTICS = CallbackTask("GET_DIAGNOSTICS", self.get_fake_diagnostics, tree=self.tree, announce=True)
       
        FAKE_CHARGING = FakeCharging("FAKE_CHARGING", self.blackboard, interval=3, announce=True)
        AUTO_DOCK = SimpleActionTask("AUTO_DOCK", "move_base", MoveBaseAction, dock_goal, tree=self.tree, done_cb=self.fake_autodock_cb, reset_after=False)
        RECHARGE_COMPLETE = ServiceTask("RECHARGE_COMPLETE", "/battery_simulator/set_battery_level", SetBatteryLevel, self.full_charge, result_cb=self.fake_recharge_cb)
        RECHARGE = Sequence("RECHARGE", [NAV_NEAR_DOCK, AUTO_DOCK, FAKE_CHARGING, RECHARGE_COMPLETE], tree=self.tree, reset_after=True)
        MONITOR_BATTERY = MonitorTask("MONITOR_BATTERY", "battery_level", Float32, self.monitor_fake_battery, tree=self.tree)
        CHECK_BATTERY_OK = CallbackTask("BATTERY_OK?", self.check_fake_battery_cb, announce=True)
        FINISH_DOCK_REQUEST = CallbackTask("FINISH_DOCK_REQUEST", self.finish_fake_doc_request, announce=True)

        STAY_HEALTHY.add_child(CHECK_BATTERY_OK)
        STAY_HEALTHY.add_child(RECHARGE)
        
        # Allow requesting a dock by publishing an empty message on topic /request_dock
        MONITOR_DOCK_REQUEST = MonitorTask("MONITOR_DOCK_REQUEST", "request_dock", Dock, self.monitor_dock_request_cb, tree=self.tree, wait_for_message=False)

        CHECK_DOCK_REQUESTED = CallbackTask("DOCK_REQUESTED?", self.check_dock_request_cb, reset_after=False)
        
        NAV_NEAR_DOCK_2 = copy.copy(NAV_NEAR_DOCK)
        NAV_NEAR_DOCK_2.name = "NAV_NEAR_DOCK_2"
 
        AUTO_DOCK_2 = copy.copy(AUTO_DOCK)
        AUTO_DOCK_2.name = "AUTO_DOCK_2"

        GOTO_DOCK = Sequence("GOTO_DOCK", [NAV_NEAR_DOCK_2, AUTO_DOCK_2, FINISH_DOCK_REQUEST], tree=self.tree, reset_after=True)
                  
        # A sequence to handle a dock request
        HANDLE_DOCK_REQUEST = Sequence("HANDLE_DOCK_REQUEST", [CHECK_DOCK_REQUESTED, GOTO_DOCK], tree=self.tree, reset_after=False)
        
        CHECK_USER_REQUESTS.add_child(HANDLE_DOCK_REQUEST)
                
        # Add the move_base tasks to the patrol task
        for task in MOVE_BASE_TASKS:
            PATROL.add_child(task)
  
        # Create the loop patrol decorator
        LOOP_PATROL = Loop("LOOP_PATROL", child=PATROL, iterations=self.n_patrols, tree=self.tree)
        
        # Add the subtrees to the root node in order of priority
        BEHAVE.add_child(GET_DIAGNOSTICS)
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(Invert("NO_USER_REQUESTS", CHECK_USER_REQUESTS))
        BEHAVE.add_child(LOOP_PATROL)
                
        # Display the tree before beginning execution
        print "Patrol Behavior Tree"
        print_tree(self.tree.root, use_symbols=True)
        print_dot_tree(self.tree.root, dotfilepath)
        
        marker_pub_interval = 2
        marker_pub_timer = 0
        self.pub_markers()
                
        # Run the tree
        while not rospy.is_shutdown():
            self.tree.status = self.tree.tick()

            print "\n *** TICK *** \n"
                        
            tick_rate.sleep()
            
            print_dot_tree(self.tree.root, dotfilepath)
                        
            marker_pub_timer += 1.0 / rate
            if marker_pub_timer > marker_pub_interval:
                self.pub_markers()
                marker_pub_timer = 0
                
    def fake_on_dock_cb(self):
        self.blackboard.on_dock = True
                
    def pub_markers(self):
        # Publish the waypoint markers
        self.marker_pub.publish(self.waypoint_markers)
        
        # Publish the docking station marker
        self.docking_station_marker_pub.publish(self.docking_station_marker)
             
        # Publish the pre dock position marker
        self.pre_dock_marker_pub.publish(self.pre_dock_marker)
            

    def get_fake_diagnostics(self):
        if self.fake:
            self.blackboard.laptop_charging = False
                        
            return True

    def check_fake_battery_cb(self):
        # If we're charging or on dock return false
        if self.blackboard.robot_charging and not self.blackboard.robot_charging_complete:
            return False
        
        if self.blackboard.robot_battery_level is None:
            return
        else:
            if self.blackboard.robot_battery_level < self.low_battery_threshold:
                rospy.logdebug("LOW BATTERY - level: " + str(int(self.blackboard.robot_battery_level)) + "%")
                return False
            else:
                rospy.logdebug("BATTERY LEVEL: " + str(int(self.blackboard.robot_battery_level)) + "%")
                return True
    
    def fake_recharge_cb(self, result):
        self.blackboard.robot_charging = False
        self.blackboard.dock_requested = False
        rospy.sleep(1)
        rospy.loginfo("FAKE RECHARGING COMPLETE!")
        return True
    
    def fake_autodock_cb(self, result_state, state):
        self.blackboard.charging = True

    def finish_fake_doc_request(self):
        self.blackboard.dock_requested = False
        return True
    
    def monitor_fake_battery(self, msg):
        # Store the fale robot battery level as published on the fake battery level topic
        self.blackboard.robot_battery_level = msg.data
        return True
    
    def monitor_dock_request_cb(self, msg):
        requests = ["DOCK", "UNDOCK", "CANCEL DOCK", "CANCEL UNDOCK"]
        
        rospy.loginfo(str(requests[msg.value]) + " REQUEST!")
        
        if msg.value == Dock.DOCK:   
            self.blackboard.dock_requested = True

        elif msg.value == Dock.UNDOCK:
            self.blackboard.undock_requested = True

        elif msg.value == Dock.CANCEL_DOCK:
            self.blackboard.dock_requested = False
            
        elif msg.value == Dock.CANCEL_UNDOCK:
            self.blackboard.undock_requested = False
        
        return True
    
    def check_dock_request_cb(self):
        if self.blackboard.dock_requested:
            return True
        else:
            return False
            
    def init_blackboard(self, blackboard):
        if blackboard.fake:
            blackboard.robot_battery_level = self.full_charge
            blackboard.robot_low_battery_alert = False
            blackboard.laptop_battery_level = self.full_charge
            blackboard.robot_battery_readings = list()
            blackboard.robot_charging = False
            blackboard.robot_charging_complete = None
            blackboard.on_dock = False
            blackboard.dock_requested = False
            blackboard.undock_requested = False
            blackboard.requested_dock_complete = False
            blackboard.wall_hit = False
        else:
            blackboard.robot_battery_level = None
            blackboard.robot_low_battery_alert = None
            blackboard.laptop_battery_level = None
            blackboard.robot_battery_readings = list()
            blackboard.robot_charging = None
            blackboard.on_dock = None
            blackboard.dock_requested = False
            blackboard.undock_requested = False
            blackboard.requested_dock_complete = False
            blackboard.wall_hit = None

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
class FakeCharging(Action):
    def __init__(self, name, blackboard, interval=3, announce=True):
        super(FakeCharging, self).__init__(name)
       
        self.name = name
        self.interval = interval
        self.blackboard = blackboard
                
        self.start_time = None
         
    def run(self):
        if self.start_time is None:
            self.start_time = time.time()
            
        print "FAKE CHARGING STATUS", self.status

        if self.status != TaskStatus.SUCCESS:
            rospy.loginfo("FAKE RECHARGING THE ROBOT...")
            rospy.sleep(0.5)
            
            print "CHARGE TIME", time.time() - self.start_time
            
            if time.time() - self.start_time < self.interval:
                return TaskStatus.RUNNING
            else:
                self.start_time = None
                self.status = TaskStatus.SUCCESS

        return self.status
    
    def reset(self):
        self.status = None
        self.start_time is None

if __name__ == '__main__':
    tree = Patrol()

