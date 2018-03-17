#!/usr/bin/env python

"""
    piko_executive.py - Version 0.1 2016-09-17
    
    Executive to control all of Pi Robot's behavior.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2016 Patrick Goebel.  All rights reserved.

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
import os

class PikoExecutive():
    def __init__(self):
        rospy.init_node('piko_executive')
        
        # Connect to the joint services and topics.
        self.init_joint_control()
        
        # Tilt head down so that obstacles are detected.
        #self.move_head(0.0, 0.707)
                
    def init_joint_control(self):
        # The list of joints is stored in the /arbotix/joints parameter
        self.joints = rospy.get_param('/arbotix/joints', '')
        
        # Select a joint to use to test the /joint_state topic
        self.test_joint = rospy.get_param('~test_joint', 'head_pan_joint')
        
        # Set a moderate default servo speed
        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.5)
                
        # A dictionary to hold the set_speed services
        self.set_joint_speed = dict()
        
        # A dictionary to hold the position publishers
        self.set_joint_position = dict()
        
        # Connect position topic and set_speed service for each joint
        for joint in self.joints:   
            speed_service = '/' + joint + '/set_speed'
            rospy.wait_for_service(speed_service)  
            self.set_joint_speed[joint] = rospy.ServiceProxy(speed_service, SetSpeed)
            
            # The position controllers
            self.set_joint_position[joint] = rospy.Publisher('/' + joint + '/command', Float64, queue_size=5)
            
        # Initialize the joint state
        self.joint_state = JointState()
            
        # Monitor the joint states topic
        rospy.loginfo("Attempting to detect joints...")
        
        try:
            # Wait until we detect the joint_states topic
            rospy.wait_for_message('joint_states', JointState, timeout=30)

            # Subscribe to the joint states topic
            rospy.Subscriber('joint_states', JointState, self.update_joint_state)
            
            # Double check that we actually have position values
            while self.joint_state == JointState():
                rospy.sleep(0.1)

            joints_detected = True
            rospy.loginfo("Joints detected.")
        except:
            rospy.loginfo("Joints not detected.")
            joints_detected = False
                    
        if not joints_detected:
            rospy.loginfo("Aborting because joints not detected!")
            os._exit(1)

        # Set the joint speed to the default value and center each joint
        for joint in self.joints:
            self.set_joint_speed[joint](self.default_joint_speed)
            self.set_joint_position[joint].publish(0)
            
        
    def move_joint(self, positions):
        n_joints = len(self.joints)
#         while any(abs(p) > 0.05 for p in self.joint_state.position):
#             for i in :
#                 set_position.publish(0)
#                 rospy.sleep(0.5)            
            
    def update_joint_state(self, msg):
        try:
            test = msg.name.index(self.test_joint)
            self.joint_state = msg
        except:
            pass

        
if __name__=='__main__':
    try:
        PikoExecutive()
        rospy.loginfo("Executive started.")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Oops! Exception occurred while trying to launch executive.")
