#!/usr/bin/env python

"""
    people_monitor.py - Version 1.0 2013-03-18
    
    Interact with people as detected and tracked using a depth camera.
    
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
from std_msgs.msg import Float32
from pi_trees_ros.pi_trees_ros import *

# A class to track global variables
class BlackBoard():
    def __init__(self):
        self.people_present = dict()    # Name and position
        self.last_seen_at = dict()      # Name and time
        self.attending_to = None        # Name

# Initialize the black board
blackboard = BlackBoard()

class PeopleMonitor():
    def __init__(self):
        rospy.init_node("people_monitor")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Create the root node
        BEHAVE = Sequence("BEHAVE")
        
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY")
        
        # Create the patrol loop decorator
        LOOP_PATROL = Loop("LOOP_PATROL", iterations=self.n_patrols)
        
        # Add the two subtrees to the root node in order of priority
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(LOOP_PATROL)
        
        # Create the patrol iterator
        PATROL = Iterator("PATROL")
        
        # Add the move_base tasks to the patrol task
        for task in MOVE_BASE_TASKS:
            PATROL.add_child(task)
  
        # Add the patrol to the loop decorator
        LOOP_PATROL.add_child(PATROL)
        
        # Add the battery check and recharge tasks to the "stay healthy" task
        with STAY_HEALTHY:
            # The check battery condition (uses MonitorTask)
            CHECK_BATTERY = MonitorTask("CHECK_BATTERY", "battery_level", Float32, self.check_battery)
            
            # The charge robot task (uses ServiceTask)
            CHARGE_ROBOT = ServiceTask("CHARGE_ROBOT", "battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb=self.recharge_cb)
      
            # Build the recharge sequence using inline construction
            RECHARGE = Sequence("RECHARGE", [NAV_DOCK_TASK, CHARGE_ROBOT])
                
            # Add the check battery and recharge tasks to the stay healthy selector
            STAY_HEALTHY.add_child(CHECK_BATTERY)
            STAY_HEALTHY.add_child(RECHARGE)
                
        # Display the tree before beginning execution
        print "Patrol Behavior Tree"
        print_tree(BEHAVE)
        
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)
            
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
        return TaskStatus.SUCCESS
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    tree = PeopleMonitor()

