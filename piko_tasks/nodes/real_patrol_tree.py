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
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from rbx2_msgs.srv import *
from pi_trees_ros3.pi_trees_ros3 import *
from pi_trees_lib3 import *
from piko_msgs.msg import Dock
from piko_tasks.task_setup import *
from diagnostic_msgs.msg import DiagnosticArray
import thread
import time
import copy
import os

# Autodocking stuff
import actionlib
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from actionlib_msgs.msg import GoalStatus

class Patrol():
    def __init__(self):
        rospy.init_node("patrol_tree", log_level=rospy.DEBUG)
        #rospy.init_node("patrol_tree")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # How frequently do we "tic" the tree?
        rate = rospy.get_param('~rate', 10)
        
        # Convert tick rate to a ROS rate
        tick_rate = rospy.Rate(rate)
        
        # Where should the DOT file be stored.  Default location is $HOME/.ros/tree.dot
        dotfilepath = rospy.get_param('~dotfilepath', None)

        # What do we consider a full charge?
        self.full_charge = rospy.get_param('~full_charge', 100)
        
        # A lock to use in various callbacks
        self.lock = thread.allocate_lock()
        
        # Create a publisher to reset the Kobuki's odometry when we know we are on the docking station
        self.reset_odom_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=5)
        
        # Create a publisher to reset the robot's AMCL position when we know we are on the docking station
        self.reset_amcl_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        
        # Initialize the blackboard
        self.blackboard = Blackboard("blackboard", debug=False)
        
        self.init_blackboard(self.blackboard)
        
        # The root node
        BEHAVE = Sequence("BEHAVE")
        
        # The behavior tree
        self.tree = BehaviorTree("Patrol Tree", root=BEHAVE, blackboard=self.blackboard, debug=False)
        
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY", tree=self.tree, announce=True)
        
        # Check for user requests
        HANDLE_USER_REQUESTS = Selector("HANDLE_USER_REQUESTS", tree=self.tree, announce=True)
        
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
    
        # Reset the Kobuki's odometry when we know we're on the docking station
        RESET_ODOMETRY = ResetOdometry("RESET_ODOMETRY", self.reset_odom_pub)
        
        # Reset the robots's AMCL pose when we know we're on the docking station
        RESET_AMCL_POSE = ResetAMCLPose("RESET_AMCL_POSE", self.reset_amcl_pub, self.docking_station_pose)
                
        # Get the current diagnostics and set a number of flags related to battery charge, etc.
        GET_DIAGNOSTICS = MonitorTask("GET_DIAGNOSTICS", "diagnostics_agg", DiagnosticArray, self.get_diagnostics)
            
        # Check to see if we're on the docking station
        CHECK_ON_DOCK = CheckBlackboard("CHECK_ON_DOCK", key='on_dock', value=True)
        
        # Check to see if we're off the docking station
        CHECK_OFF_DOCK = CheckBlackboard("CHECK_OFF_DOCK", key='on_dock', value=False)
        
        # Use a callback to move backward off the docking station
        BACK_OFF_DOCK = BackOffDock("BACK_OFF_DOCK", self.cmd_vel_pub, interval=6)
        
        # A selector to manage backing off the docking station
        GET_OFF_DOCK = Selector("GET_OFF_DOCK", [CHECK_OFF_DOCK, Sequence(None, [RESET_ODOMETRY, BACK_OFF_DOCK], reset_after=True)])
        
        #IF_ON_DOCK = ConditionalTask("IF_ON_DOCK", GET_OFF_DOCK, self.tree.blackboard.memory['on_dock'])
                                         
        # Connect to the Kobuki autodock action server
        AUTO_DOCK = SimpleActionTask("AUTO_DOCK", "dock_drive_action", AutoDockingAction, AutoDockingGoal(), tree=self.tree, connect_timeout=10, result_timeout=300, feedback_cb=self.autodock_feedback_cb, done_cb=self.autodock_done_cb, reset_after=False)
        
        RECHARGE = Sequence("RECHARGE", [CHECK_OFF_DOCK, NAV_NEAR_DOCK, AUTO_DOCK], tree=self.tree, reset_after=True)
        CHECK_BATTERY_OK = CallbackTask("BATTERY_OK?", cb=self.check_battery_cb, tree=self.tree, announce=True)
        
        # Build up the STAY_HEALTHY selector
        STAY_HEALTHY.add_child(CHECK_BATTERY_OK)
        STAY_HEALTHY.add_child(RECHARGE)
        
        # Allow requesting a dock by publishing an empty message on topic /request_dock
        MONITOR_DOCK_REQUEST = MonitorTask("MONITOR_DOCK_REQUEST", "request_dock", Dock, self.monitor_dock_request_cb, tree=self.tree, wait_for_message=False)

        CHECK_DOCK_REQUESTED = CallbackTask("DOCK_REQUESTED?", cb=self.check_dock_request_cb, tree=self.tree, reset_after=False)
        
        FINISH_DOCK_REQUEST = FinishDockRequest("FINISH_DOCK_REQUEST", interval=3)
        
#         NAV_NEAR_DOCK_2 = copy.copy(NAV_NEAR_DOCK)
#         NAV_NEAR_DOCK_2.name = "NAV_NEAR_DOCK_2"
#  
#         AUTO_DOCK_2 = copy.copy(AUTO_DOCK)
#         AUTO_DOCK_2.name = "AUTO_DOCK_2"

        GOTO_DOCK = Sequence("GOTO_DOCK", [CHECK_OFF_DOCK, NAV_NEAR_DOCK, AUTO_DOCK], tree=self.tree, reset_after=True)
                  
        # A sequence to handle a dock request
        HANDLE_DOCK_REQUEST = Sequence("HANDLE_DOCK_REQUEST", [CHECK_DOCK_REQUESTED, GOTO_DOCK], tree=self.tree, reset_after=False)
        
        HANDLE_USER_REQUESTS.add_child(HANDLE_DOCK_REQUEST)
                
        # Add the move_base tasks to the patrol task
        for task in MOVE_BASE_TASKS:
            PATROL.add_child(task)
            
        PREPARE_PATROL = Sequence("PREPARE_PATROL", [GET_OFF_DOCK, PATROL])
  
        # Create the loop patrol decorator
        LOOP_PATROL = Loop("LOOP_PATROL", child=PREPARE_PATROL, iterations=self.n_patrols, tree=self.tree)
        
        # Add the subtrees to the root node in order of priority
        BEHAVE.add_child(GET_DIAGNOSTICS)
        #BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(Invert("USER_REQUESTS", HANDLE_USER_REQUESTS))
        BEHAVE.add_child(LOOP_PATROL)
        #BEHAVE.add_child(GOTO_DOCK)
                
        # Display the tree before beginning execution
        print "Patrol Behavior Tree"
        print_tree(self.tree.root, use_symbols=True)
        print_dot_tree(self.tree.root, dotfilepath)
        
        marker_pub_interval = 2
        marker_pub_timer = 0
        self.pub_markers()
                
        ''' ***** Run the tree ***** '''
        while not rospy.is_shutdown():
            self.tree.status = self.tree.tick()

            #print "\n *** TICK *** \n"
                        
            tick_rate.sleep()
            
            print_dot_tree(self.tree.root, dotfilepath)
                        
            marker_pub_timer += 1.0 / rate
            if marker_pub_timer > marker_pub_interval:
                self.pub_markers()
                marker_pub_timer = 0
                
    def pub_markers(self):
        # Publish the waypoint markers
        self.marker_pub.publish(self.waypoint_markers)
        
        # Publish the docking station marker
        self.docking_station_marker_pub.publish(self.docking_station_marker)
             
        # Publish the pre dock position marker
        self.pre_dock_marker_pub.publish(self.pre_dock_marker)
            
    def check_battery_cb(self):
        # If we're charging or on dock, return False
        if self.blackboard.memory['robot_charging'] or self.blackboard.memory['on_dock']:
            return False
        
        if self.blackboard.robot_battery_level is None:
            return
        else:
            if self.blackboard.memory['robot_low_battery_alert'] or self.blackboard.memory['robot_battery_level'] < self.low_battery_threshold:
                self.blackboard.memory['robot_low_battery_alert'] = True
                rospy.logdebug("LOW BATTERY - level: " + str(int(self.blackboard.memory['robot_battery_level'])) + "%")
                return False
            else:
                rospy.logdebug("BATTERY LEVEL: " + str(int(self.blackboard.memory['robot_battery_level'])) + "%")
                return True
    
    def check_dock_request_cb(self):
        if self.blackboard.memory['dock_requested']:
            return True
        else:
            return False
        
    def monitor_dock_request_cb(self, msg):
        requests = ["DOCK", "UNDOCK", "CANCEL DOCK", "CANCEL UNDOCK"]
        
        rospy.loginfo(str(requests[msg.value]) + " REQUEST!")
        
        self.lock.acquire()
        
        if msg.value == Dock.DOCK:   
            self.blackboard.memory['dock_requested'] = True

        elif msg.value == Dock.UNDOCK:
            self.blackboard.memory['undock_requested'] = True

        elif msg.value == Dock.CANCEL_DOCK:
            self.blackboard.memory['dock_requested'] = False
            
        elif msg.value == Dock.CANCEL_UNDOCK:
            self.blackboard.memory['undock_requested'] = False
            
        self.lock.release()
        
        return True
        
    def get_diagnostics(self, msg):
        # Code taken from kobuki_dashboard package

        laptop_battery_status = {}
        
        for status in msg.status:
            if status.name == "/Kobuki/Motor State":
                motor_state = int(status.values[0].value)
                #rospy.logdebug("MOTOR STATE: " + str(motor_state))

            elif status.name == "/Power System/Battery":
                for value in status.values:
                    if value.key == 'Percent':
                        #self.blackboard.memory['robot_battery_readings'].append(float(value.value))
                        #self.blackboard.memory['robot_battery_level'] = moving_average(self.blackboard.memory['robot_battery_readings'], 60)
                        self.blackboard.memory['robot_battery_level'] = float(value.value)
                        
                        #rospy.logdebug("BATTERY PERCENT: " + str(self.blackboard.memory['robot_battery_level']))
                    elif value.key == "Charging State":
                        if value.value == "Trickle Charging" or value.value == "Full Charging":
                            self.blackboard.memory['robot_charging'] = True
                            self.blackboard.memory['on_dock'] = True
                        else:
                            self.blackboard.memory['robot_charging'] = False
                            self.blackboard.memory['on_dock'] = False
                        #rospy.logdebug("ROBOT CHARGING: " + str(self.blackboard.memory['robot_charging']))

            elif status.name == "/Power System/Laptop Battery":
                for value in status.values:
                    laptop_battery_status[value.key] = value.value
                    
#             elif status.name == "/Sensors":
#                 for value in status.values:
#                     if value.key == 'mobile_base_nodelet_manager: Wall Sensor':
#                         if value.value == 'Wall Hit!':
#                             self.blackboard.memory['wall_hit'] = True
#                         else:
#                             self.blackboard.memory['wall_hit'] = False
#         
#         if self.blackboard.memory['robot_charging'] and self.blackboard.memory['wall_hit']:
#             self.blackboard.memory['on_dock'] = True
#         else:
#             self.blackboard.memory['on_dock'] = False
            
        if (laptop_battery_status):
            percentage = 100. * float(laptop_battery_status['Charge (Ah)'])/float(laptop_battery_status['Capacity (Ah)'])
            self.blackboard.memory['laptop_battery_level'] = percentage
            #rospy.logdebug("LAPTOP BATTERY: " + str(self.blackboard.memory['laptop_battery_level']))

            charging_state = True if float(laptop_battery_status['Current (A)']) > 0.0 else False
            self.blackboard.memory['laptop_charging'] = charging_state
            
            #rospy.logdebug("LAPTOP CHARGING: " + str(self.blackboard.memory['laptop_charging']))

        return True
    
    def autodock_done_cb(self, status, result):
        self.lock.acquire()
        self.blackboard.memory['dock_requested'] = False
        self.lock.release()
        
        if 0: print ''
        elif status == GoalStatus.PENDING   : state='PENDING'
        elif status == GoalStatus.ACTIVE    : state='ACTIVE'
        elif status == GoalStatus.PREEMPTED : state='PREEMPTED'
        elif status == GoalStatus.SUCCEEDED : state='SUCCEEDED'
        elif status == GoalStatus.ABORTED   : state='ABORTED'
        elif status == GoalStatus.REJECTED  : state='REJECTED'
        elif status == GoalStatus.PREEMPTING: state='PREEMPTING'
        elif status == GoalStatus.RECALLING : state='RECALLING'
        elif status == GoalStatus.RECALLED  : state='RECALLED'
        elif status == GoalStatus.LOST      : state='LOST'

        rospy.logdebug("Result - [ActionServer: " + str(state) + "]: " + str(result.text))
          
    def autodock_active_cb(self):
        if 0: rospy.logdebug("Action server went active.")
    
    def autodock_feedback_cb(self, feedback):
        # Print state of dock_drive module (or node.)
        rospy.logdebug("Feedback: [DockDrive: " + str(feedback.state) + "]: " + str(feedback.text))
        
        if feedback.state == 'DOCKED_IN':
            self.blackboard.memory['on_dock'] == True
            
    def init_blackboard(self, blackboard):
        blackboard.memory['robot_battery_level'] = None
        blackboard.memory['robot_low_battery_alert'] = None
        blackboard.memory['laptop_battery_level'] = None
        blackboard.memory['robot_battery_readings'] = list()
        blackboard.memory['robot_charging'] = None
        blackboard.memory['dock_requested'] = False
        blackboard.memory['undock_requested'] = False
        blackboard.memory['requested_dock_complete'] = False
        blackboard.memory['wall_hit'] = None
        self.blackboard.memory['on_dock'] = None

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        
class BackOffDock(Action):
    def __init__(self, name, cmd_vel_pub, interval=3):
        super(BackOffDock, self).__init__(name)
       
        self.name = name
        self.interval = interval
        self.cmd_vel_pub = cmd_vel_pub
        
        self.timer = 0
         
    def run(self):
        if self.status != TaskStatus.SUCCESS:
            rospy.logdebug("LEAVING THE DOCKING STATION")
            
            while self.timer < self.interval:
                cmd_vel = Twist()
                cmd_vel.linear.x = -0.05
            
                self.cmd_vel_pub.publish(cmd_vel)
                self.timer += 0.1
                rospy.sleep(0.1)
                
                self.status = TaskStatus.RUNNING

            self.cmd_vel_pub.publish(Twist())
            self.timer = 0
            self.status = TaskStatus.SUCCESS
        
        return self.status
    
class FinishDockRequest(Action):
    def __init__(self, name, interval=3):
        super(FinishDockRequest, self).__init__(name)
       
        self.name = name
        self.interval = interval        
        self.timer = 0
         
    def run(self):
        if self.status != TaskStatus.SUCCESS:
            rospy.logdebug("FINISHING DOCK REQUEST")
            
            if self.timer < self.interval:
                self.timer += 0.1
                time.sleep(0.1)
                self.status = TaskStatus.RUNNING
            else:
                self.blackboard.memory['dock_requested'] = False
                self.status = TaskStatus.SUCCESS
        
        return self.status
        
class ResetOdometry(Action):
    def __init__(self, name, reset_odom_pub):
        super(ResetOdometry, self).__init__(name)
       
        self.name = name
        self.reset_odom_pub = reset_odom_pub
         
    def run(self):
        if self.status != TaskStatus.SUCCESS:
            rospy.logdebug("RESETTING ODOMETRY")
            self.reset_odom_pub.publish(Empty())

            self.status = TaskStatus.SUCCESS
            
        return self.status
    
class ResetAMCLPose(Action):
    def __init__(self, name, reset_amcl_pub, reset_pose):
        super(ResetAMCLPose, self).__init__(name)
       
        self.name = name
        self.reset_amcl_pub = reset_amcl_pub
        self.initialpose = PoseWithCovarianceStamped
        self.reset_pose.header.frame_id = '/map'
        self.reset_pose.header.stamp = rospy.Time.now()
        self.reset_pose.pose.pose = reset_pose
         
    def run(self):
        if self.status != TaskStatus.SUCCESS:
            rospy.logdebug("RESETTING AMCL POse")
            self.reset_odom_pub.publish(Empty())

            self.status = TaskStatus.SUCCESS
            
        return self.status

if __name__ == '__main__':
    tree = Patrol()

