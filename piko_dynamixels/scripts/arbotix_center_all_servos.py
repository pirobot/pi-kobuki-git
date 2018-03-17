#!/usr/bin/env python

"""
    arbotix_center_all_servos.py - Version 0.1 2012-03-24
    
    Move all servos to position 0.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2015 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from arbotix_msgs.srv import SetSpeed
from std_srvs.srv import Empty
import os

class CenterServos():
    def __init__(self):
        rospy.init_node('center_all_servos')
        
        # The list of joints is stored in the /arbotix/joints parameter
        self.joints = rospy.get_param('/arbotix/joints', '')
        
        self.test_joint = rospy.get_param('~test_joint', 'head_pan_joint')
        
        # Set a moderate default servo speed
        default_speed = rospy.get_param('~default_speed', 0.5)
                
        # A list to hold the set_speed services
        set_speed_services = list()
        
        # A list to hold the relax services
        relax_services = list()
        
        # A list to hold the position publishers
        set_position_publishers = list()
        
        # Connect position topic and set_speed service for each joint
        for joint in self.joints:   
            speed_service = '/' + joint + '/set_speed'
            rospy.wait_for_service(speed_service)  
            set_speed_services.append(rospy.ServiceProxy(speed_service, SetSpeed))
            
            relax_service = '/' + joint + '/relax'
            rospy.wait_for_service(relax_service)  
            relax_services.append(rospy.ServiceProxy(relax_service, Empty))
            
            # The position controllers
            set_position_publishers.append(rospy.Publisher('/' + joint + '/command', Float64, queue_size=5))
            
        # Initialize the joint state
        self.joint_state = JointState()
            
        # Monitor the joint states of the servos
        rospy.loginfo("Attempting to detect servos...")
        
        try:
            # Wait until we detect the joint_states topic
            rospy.wait_for_message('joint_states', JointState, timeout=30)

            # Subscribe to the joint states topic
            rospy.Subscriber('joint_states', JointState, self.update_joint_state)
            
            # Double check that we actually have position values
            while self.joint_state == JointState():
                rospy.sleep(0.1)

            servos_detected = True
            rospy.loginfo("Servos detected.")
        except:
            rospy.loginfo("Servos not detected.")
            servos_detected = False
                    
        if not servos_detected:
            rospy.loginfo("Aborting center servos script")
            os._exit(1)

        # Set the servo speed to the default value
        for set_speed in set_speed_services:
            set_speed(default_speed)
                        
        # Set each servo to the 0 position
        while any(abs(p) > 0.05 for p in self.joint_state.position):
            for set_position in set_position_publishers:
                set_position.publish(0)
                rospy.sleep(0.5)
                
        # Relax each servo
        for relax in relax_services:
            relax()
            
        # Do it again just to be sure
        for relax in relax_services:
            relax()       
            
    def update_joint_state(self, msg):
        try:
            test = msg.name.index(self.test_joint)
            self.joint_state = msg
        except:
            pass

        
if __name__=='__main__':
    try:
        CenterServos()
        rospy.loginfo("All servos centered.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Oops! Exception occurred while trying to center servos...")
