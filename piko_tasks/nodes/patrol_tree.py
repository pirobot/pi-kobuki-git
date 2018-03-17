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
from pi_trees_lib3 import *
from pi_trees_ros3.pi_trees_ros3 import *
from piko_msgs.msg import Dock
from piko_tasks.task_setup import *
from diagnostic_msgs.msg import DiagnosticArray
import thread
import time

# Autodocking stuff
import actionlib
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from actionlib_msgs.msg import GoalStatus

# Reserve a thread lock
thread_lock = thread.allocate_lock()

# A class to track global variables
class BlackBoard():
    def __init__(self, name, fake):
        self.memory = {}
        
#         if fake:
#             self.memory['robot_battery_level'] = 100
#             self.memory['robot_low_battery_alert'] = False
#             self.memory['laptop_battery_level'] = 100
#             self.memory['robot_battery_readings'] = list()
#             self.memory['robot_charging'] = False
#             self.memory['on_dock'] = False
#             self.memory['dock_requested'] = False
#             self.memory['undock_requested'] = False
#             self.memory['undock_complete'] = None
#             self.memory['requested_dock_complete'] = None
#             self.memory['wall_hit'] = False
#             self.node_status = {}
#         else:
#             self.memory['robot_battery_level'] = None
#             self.memory['robot_low_battery_alert'] = None
#             self.memory['laptop_battery_level'] = None
#             self.memory['robot_battery_readings'] = list()
#             self.memory['robot_charging'] = None
#             self.memory['on_dock'] = None
#             self.memory['dock_requested'] = False
#             self.memory['undock_requested'] = False
#             self.memory['undock_complete'] = None
#             self.memory['requested_dock_complete'] = None
#             self.memory['wall_hit'] = None
#             self.node_status = {}
            
        if fake:
            self.robot_battery_level = 100
            self.robot_low_battery_alert = False
            self.laptop_battery_level = 100
            self.robot_battery_readings = list()
            self.robot_charging = False
            self.on_dock = False
            self.dock_requested = False
            self.undock_requested = False
            self.requested_dock_complete = False
            self.wall_hit = False
        else:
            self.robot_battery_level = None
            self.robot_low_battery_alert = None
            self.laptop_battery_level = None
            self.robot_battery_readings = list()
            self.robot_charging = None
            self.on_dock = None
            self.dock_requested = False
            self.undock_requested = False
            self.requested_dock_complete = False
            self.wall_hit = None

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
        BEHAVE = Sequence("Behave")
        
        # The behavior tree
        self.tree = BehaviorTree("Patrol Tree", root=BEHAVE, blackboard=self.blackboard)
        
        # Create a publisher to reset the Kobuki's odometry when we know we are on the docking station
        self.reset_odom_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=5)
            
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
        
        WAIT30 = WaitTask("WAIT30", interval=30)
        
        if self.fake:
            FAKE_CHARGING = FakeCharging("FAKE_CHARGING", self.blackboard, interval=3, announce=True)
            AUTO_DOCK = SimpleActionTask("AUTO_DOCK", "move_base", MoveBaseAction, dock_goal, tree=self.tree, reset_after=False)
            GOTO_DOCK = Sequence("GOTO_DOCK", [NAV_NEAR_DOCK, AUTO_DOCK], tree=self.tree, announce=True)
            RECHARGE_COMPLETE = ServiceTask("RECHARGE_COMPLETE", "/battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb=self.fake_recharge_cb)
            RECHARGE = Sequence("RECHARGE", [GOTO_DOCK, FAKE_CHARGING, RECHARGE_COMPLETE], tree=self.tree, reset_after=True)
            MONITOR_BATTERY = MonitorTask("MONITOR_BATTERY", "battery_level", Float32, self.monitor_fake_battery, tree=self.tree)
        else:
            # Connect to the Kobuki autodock action server
            AUTO_DOCK = SimpleActionTask("AUTO_DOCK", "dock_drive_action", AutoDockingAction, AutoDockingGoal(), connect_timeout=10, result_timeout=300, feedback_cb=self.autodock_feedback_cb, done_cb=self.autodock_done_cb, reset_after=False)
            
            # Check to see if we're OFF the docking station
            CHECK_OFF_DOCK1 = CallbackTask("OFF_DOCK 1?", self.check_off_dock_cb)
            
            CHECK_OFF_DOCK2 = CallbackTask("OFF_DOCK 2?", self.check_off_dock_cb)
            
            # Check to see if we're ON the docking station
            CHECK_ON_DOCK = CallbackTask("ON_DOCK?", self.check_on_dock_cb)
            
            # A task to assert that a requested dock is complete
            REQUESTED_DOCK_COMPLETE = SetVariable("REQUESTED_DOCK_COMPLETE", self.blackboard, 'requested_dock_complete', True)
            
            # Go to the docking station by moving near to the dock then calling the autodock action
            GOTO_DOCK = Sequence("GOTO_DOCK", [NAV_NEAR_DOCK, AUTO_DOCK, REQUESTED_DOCK_COMPLETE], announce=True, reset_after=True)
            
            # Recharge the robot by going to the docking station
            #RECHARGE = Sequence ("RECHARGE", [GOTO_DOCK], reset_after=True)
        
        if self.fake:
            # Get the fake diagnostics like battery charge.
            GET_DIAGNOSTICS = CallbackTask("GET_DIAGNOSTICS", self.get_fake_diagnostics, tree=self.tree, announce=True)

        else:
            # Get the current diagnostics and set a number of flags related to battery charge, etc.
            GET_DIAGNOSTICS = MonitorTask("GET_DIAGNOSTICS", "diagnostics_agg", DiagnosticArray, self.get_diagnostics)
        
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY", tree=self.tree, announce=True)
        
        # Flesh out the STAY_HEALTHY subtree
        with STAY_HEALTHY:
            # Check to see if the robot is charging
            CHECK_CHARGING = CallbackTask("CHARGING?", self.check_charging_cb, announce=True)
            
            # Check to see if charging is complete
            CHECK_CHARGE_COMPLETE = CallbackTask("CHARGING_COMPLETE?", self.check_charge_complete_cb, announce=True)
            
            # Check the robot battery level
            CHECK_BATTERY_OK = CallbackTask("BATTERY_OK?", self.check_battery_cb, announce=True)
            
            # Are we charging and is charging complete?
            RECHARGE_STATUS = Sequence("RECHARGE_STATUS", [CHECK_CHARGING, CHECK_CHARGE_COMPLETE], tree=self.tree, announce=True)

            # If we are not charging, is the battery level OK?
            CHECK_CHARGE = Selector("CHECK_CHARGE", [RECHARGE_STATUS, CHECK_BATTERY_OK], tree=self.tree, announce=True)
    
            # Tie together the charge/battery checks and recharge task
            STAY_HEALTHY.add_child(CHECK_CHARGE)
            STAY_HEALTHY.add_child(RECHARGE)
        
        # A selector to manage backing off the docking station
        GET_OFF_DOCK = Sequence("GET_OFF_DOCK")

        # Make sure we are off the docking station before continuing patrol
        with GET_OFF_DOCK:
            # Reset the Kobuki's odometry when we know we're on the docking station
            RESET_ODOMETRY = ResetOdometry("RESET_ODOMETRY", self.reset_odom_pub)
                        
            # Use a callback to move backward off the docking station
            BACK_OFF_DOCK = BackOffDock("BACK_OFF_DOCK", self.blackboard, self.cmd_vel_pub, interval=6)

            UNDOCK = Sequence("UNDOCK", [RESET_ODOMETRY, BACK_OFF_DOCK], tree=self.tree, reset_after=True)
        
            # If on the dock, undock
            if not self.fake:
                GET_OFF_DOCK.add_child(CHECK_ON_DOCK)
            GET_OFF_DOCK.add_child(UNDOCK)
          
        # Allow requesting a dock by publishing an empty message on topic /request_dock
        MONITOR_DOCK_REQUEST = MonitorTask("MONITOR_DOCK_REQUEST", "request_dock", Dock, self.monitor_dock_request_cb, tree=self.tree, wait_for_message=False)
          
        # The user can request a dock action by publishing on the /request_dock topic
        DOCK_REQUEST = Sequence("DOCK_REQUEST", tree=self.tree, reset_after=False, announce=True)
        
        # Invert the result of the dock request to that it will block subsequent tasks if True
        INVERT_DOCK_REQUEST = Invert("INVERT_DOCK_REQUEST", DOCK_REQUEST, tree=self.tree)
        
        INVERT_GET_OFF_DOCK = Invert("OFF_DOCK?", GET_OFF_DOCK, tree=self.tree)
                
        # A sequence to handle a dock request
        HANDLE_DOCK_REQUEST = Selector("HANDLE_DOCK_REQUEST", tree=self.tree, reset_after=False)
             
        with HANDLE_DOCK_REQUEST:
            # A task to check if the requested dock is complete
            CHECK_REQUESTED_DOCK_COMPLETE = IfNotNull("CHECK_REQUESTED_DOCK_COMPLETE", self.blackboard, 'requested_dock_complete')
               
            # Build the requested dock sequence
            HANDLE_DOCK_REQUEST.add_child(CHECK_REQUESTED_DOCK_COMPLETE)
            HANDLE_DOCK_REQUEST.add_child(GOTO_DOCK)
 
        with DOCK_REQUEST:
            # Check to see if a docking request has been made
            CHECK_DOCK_REQUEST = CallbackTask("DOCK_REQUESTED?", self.check_dock_request_cb, reset_after=False)
             
            DOCK_REQUEST.add_child(CHECK_DOCK_REQUEST)        
            DOCK_REQUEST.add_child(HANDLE_DOCK_REQUEST)
                     
        # Create the patrol iterator
        PATROL = Iterator("PATROL", tree=self.tree, reset_after=False, announce=True)
        
        #PATROL.add_child(GET_OFF_DOCK)
                
        # Add the move_base tasks to the patrol task
        for task in MOVE_BASE_TASKS:
            PATROL.add_child(task)
  
        # Create the loop patrol decorator
        LOOP_PATROL = Loop("LOOP_PATROL", child=PATROL, iterations=self.n_patrols, tree=self.tree)
        
        # Add the subtrees to the root node in order of priority
        BEHAVE.add_child(GET_DIAGNOSTICS)
        BEHAVE.add_child(STAY_HEALTHY)
        #BEHAVE.add_child(INVERT_DOCK_REQUEST)
        #BEHAVE.add_child(INVERT_GET_OFF_DOCK)
        BEHAVE.add_child(LOOP_PATROL)
                
        # Display the tree before beginning execution
        print "Patrol Behavior Tree"
        print_tree(self.tree.root, use_symbols=True)
        
        marker_pub_interval = 2
        marker_pub_timer = 0
        self.pub_markers()
                
        # Run the tree
        while not rospy.is_shutdown():
            self.tree.status = self.tree.tick()
                        
            tick_rate.sleep()
            
            print "\n *** TICK *** \n"
            
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
                        #self.blackboard.robot_battery_readings.append(float(value.value))
                        #self.blackboard.robot_battery_level = moving_average(self.blackboard.robot_battery_readings, 60)
                        self.blackboard.robot_battery_level = float(value.value)
                        
                        #rospy.logdebug("BATTERY PERCENT: " + str(self.blackboard.robot_battery_level))
                    elif value.key == "Charging State":
                        if value.value == "Trickle Charging" or value.value == "Full Charging":
                            self.blackboard.robot_charging = True
                            self.blackboard.on_dock = True
                        else:
                            self.blackboard.robot_charging = False
                            self.blackboard.on_dock = False
                        #rospy.logdebug("ROBOT CHARGING: " + str(self.blackboard.robot_charging))

            elif status.name == "/Power System/Laptop Battery":
                for value in status.values:
                    laptop_battery_status[value.key] = value.value
                    
#             elif status.name == "/Sensors":
#                 for value in status.values:
#                     if value.key == 'mobile_base_nodelet_manager: Wall Sensor':
#                         if value.value == 'Wall Hit!':
#                             self.blackboard.wall_hit = True
#                         else:
#                             self.blackboard.wall_hit = False
#         
#         if self.blackboard.robot_charging and self.blackboard.wall_hit:
#             self.blackboard.on_dock = True
#         else:
#             self.blackboard.on_dock = False
            
        if (laptop_battery_status):
            percentage = 100. * float(laptop_battery_status['Charge (Ah)'])/float(laptop_battery_status['Capacity (Ah)'])
            self.blackboard.laptop_battery_level = percentage
            #rospy.logdebug("LAPTOP BATTERY: " + str(self.blackboard.laptop_battery_level))

            charging_state = True if float(laptop_battery_status['Current (A)']) > 0.0 else False
            self.blackboard.laptop_charging = charging_state
            
            #rospy.logdebug("LAPTOP CHARGING: " + str(self.blackboard.laptop_charging))

        return True
    
    def get_fake_diagnostics(self):
        # Code taken from kobuki_dashboard package
        if self.fake:
            #self.blackboard.laptop_battery_level = 100
            self.blackboard.laptop_charging = False
            #self.blackboard.robot_low_battery_alert = False
                        
            return True
    
    def check_off_dock_cb(self):
        rospy.logdebug("CHECK OFF DOCK")
        if self.fake:
            return self.blackboard.on_dock
        else:
            if self.blackboard.robot_charging is None and self.blackboard.on_dock is None:
                return
            elif self.blackboard.robot_charging or self.blackboard.on_dock:
                rospy.logdebug("OFF DOCK: FALSE")
                return False
            else:
                rospy.logdebug("OFF DOCK: TRUE")
                return True
            
    def check_on_dock_cb(self):
        rospy.logdebug("CHECK ON DOCK")
        if self.fake:
            return self.blackboard.on_dock
        else:
            if self.blackboard.robot_charging is None and self.blackboard.on_dock is None:
                return
            elif self.blackboard.robot_charging or self.blackboard.on_dock:
                rospy.logdebug("ON DOCK: TRUE")
                return True
            else:
                rospy.logdebug("ON DOCK: FALSE")
                return False
            
    def check_battery_cb(self):
        # Don't run the check if we are charging
        if self.blackboard.robot_charging or self.blackboard.on_dock:
            return
        
        if self.blackboard.robot_battery_level is None:
            return
        else:
            if self.blackboard.robot_low_battery_alert or self.blackboard.robot_battery_level < self.low_battery_threshold:
                self.blackboard.robot_low_battery_alert = True
                rospy.logdebug("LOW BATTERY - level: " + str(int(self.blackboard.robot_battery_level)) + "%")
                return False
            else:
                rospy.logdebug("BATTERY LEVEL: " + str(int(self.blackboard.robot_battery_level)) + "%")
                return True
            
    def check_charging_cb(self):      
        rospy.logdebug("CHECK CHARGING")
        
        if self.fake:
            if self.blackboard.on_dock:
                rospy.logdebug("STILL ON DOCK")
            else:
                rospy.logdebug("OFF DOCK")

            return self.blackboard.on_dock
        
        if self.blackboard.robot_charging is None and self.blackboard.on_dock is None:
            #rospy.logdebug("UNKNOWN CHARGING STATUS...")
            return
        else:
            if self.blackboard.robot_charging or self.blackboard.on_dock:
                # Clear the robot_low_battery flag if set
                self.blackboard.robot_low_battery_alert = False
                rospy.logdebug("ROBOT CHARGING...")
                return True
            else:
                rospy.logdebug("ROBOT NOT CHARGING")
                return False
            
    def check_charge_complete_cb(self):
        rospy.logdebug("WAITING FOR CHARGING TO COMPLETE.  CHARGE AT " + str(int(self.blackboard.robot_battery_level)) + "%")
        
        if self.blackboard.robot_battery_level is None:
            return
        else:
            if self.blackboard.robot_battery_level < self.full_charge:
                return False
            else:
                rospy.logdebug("ROBOT BATTERY FULLY CHARGED!")
                return True
    
    def monitor_dock_request_cb(self, msg):
        requests = ["DOCK", "UNDOCK", "CANCEL DOCK", "CANCEL UNDOCK"]
        
        rospy.loginfo(str(requests[msg.value]) + " REQUEST!")
        
        if msg.value == Dock.DOCK:   
            self.blackboard.dock_requested = True
            self.blackboard.undock_requested = False

        elif msg.value == Dock.UNDOCK:
            self.blackboard.undock_requested = True
            self.blackboard.dock_requested = False

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
        
    def check_requested_dock_complete(self):
        rospy.loginfo("CHECK REQUESTED DOCK COMPLETE")
        
        if self.blackboard.requested_dock_complete is None:
            return None
        elif self.blackboard.requested_dock_complete:
            return True
        else:
             return False
    
    def fake_recharge_cb(self, result):
        self.blackboard.robot_low_battery_alert = False
        rospy.sleep(1)
        rospy.loginfo("RECHARGING COMPLETE!")
        return True
    
    def monitor_fake_battery(self, msg):
        # Store the fale robot battery level as published on the fake battery level topic
        self.blackboard.robot_battery_level = msg.data
        return True

    def autodock_done_cb(self, status, result):
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
        
        if feedback.state == 'DOCKED_IN' and not self.blackboard.on_dock:
            self.blackboard.on_dock = True
            
    def init_blackboard(self, blackboard):
        if blackboard.fake:
            blackboard.robot_battery_level = 100
            blackboard.robot_low_battery_alert = False
            blackboard.laptop_battery_level = 100
            blackboard.robot_battery_readings = list()
            blackboard.robot_charging = False
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
        
class BackOffDock(Action):
    def __init__(self, name, blackboard, cmd_vel_pub, interval=3):
        super(BackOffDock, self).__init__(name)
       
        self.name = name
        self.interval = interval
        self.blackboard = blackboard
        self.cmd_vel_pub = cmd_vel_pub
        
        self.timer = 0
         
    def run(self):
        if self.status != TaskStatus.SUCCESS:
            rospy.logdebug("LEAVING THE DOCKING STATION")
            if self.timer < self.interval:
                cmd_vel = Twist()
                cmd_vel.linear.x = -0.05
            
                self.cmd_vel_pub.publish(cmd_vel)
                self.timer += 0.1
                rospy.sleep(0.1)
                
                self.status = TaskStatus.RUNNING
            else:
                self.cmd_vel_pub.publish(Twist())
                self.timer = 0
                self.blackboard.on_dock = False
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
    
class FakeCharging(Action):
    def __init__(self, name, blackboard, interval=3, announce=True):
        super(FakeCharging, self).__init__(name)
       
        self.name = name
        self.interval = interval
        self.blackboard = blackboard
        
        self.start_time = 0
         
    def run(self):
        if self.start_time is None:
            self.start_time = time.time()
            
        print "FAKE CHARGING STATUS", self.status
        self.blackboard.on_dock = True

        if self.status != TaskStatus.SUCCESS:
            rospy.loginfo("FAKE RECHARGING THE ROBOT...")
            rospy.sleep(0.5)
            
            if time.time() - self.start_time < self.interval:
                self.status = TaskStatus.RUNNING
            else:
                self.start_time is None
                self.status = TaskStatus.SUCCESS

        return self.status
    
    def reset(self):
        self.status = None
        self.start_time is None
    
class SetVariable(Action):
    def __init__(self, name, blackboard, variable, value):
        super(SetVariable, self).__init__(name)
       
        self.name = name
        self.blackboard = blackboard
        self.variable = variable
        self.value = value
                 
    def run(self):
        self.blackboard.self.variable = self.value

        return TaskStatus.SUCCESS
    
class IfNotNull(Condition):
    def __init__(self, name, blackboard, variable):
        super(IfNotNull, self).__init__(name)
       
        self.name = name
        self.blackboard = blackboard
        self.variable = variable
                 
    def run(self):
        try:
            if self.blackboard.memory[self.variable] is not None:
                return TaskStatus.SUCCESS
            else:
                return TaskStatus.FAILURE
        except:
            return TaskStatus.FAILURE
    
    def reset(self):
        self.variable = None
        

if __name__ == '__main__':
    tree = Patrol()

