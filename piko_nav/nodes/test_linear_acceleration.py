#!/usr/bin/env python

""" test_accleration.py - Version 1.0 2016-8-17

    Command to robot to move from 0 to some target speed and measure acceleration.

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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pow

class TestAccleration():
    def __init__(self):
        # Give the node a name
        rospy.init_node('test_accleration', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # How fast will we update the robot's movement?
        rate = rospy.get_param('~rate', 50)
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the target linear speed
        target_speed = rospy.get_param('~target_speed', 0.5)
        
        # Set the max distance in meters
        max_distance = rospy.get_param('~max_distance', 1.0)

        # Initialize the movement command
        move_cmd = Twist()
        
        # Set the movement command to the target speed
        move_cmd.linear.x = target_speed
        
        # Keep track of the distance traveled
        distance = 0
        
        # Keep track of time elapsed
        self.start_time = None

        # Assume the start velocity is 0
        self.start_velocity = Twist()
        
        # Track max acceleration
        self.max_acceleration = 0.0
               
        # Subscribe to the /odom topic so we can measure position and accleration
        rospy.wait_for_message("odom", Odometry)
        odom_sub = rospy.Subscriber("odom", Odometry, self.get_odom)
        
        rospy.sleep(1)
        
        # Get the starting position        
        x_start = self.position.x
        y_start = self.position.y
        
        # Move until target speed or max distance is reached
        while distance < max_distance and not rospy.is_shutdown():
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((self.position.x - x_start), 2) + 
                            pow((self.position.y - y_start), 2))
            
            # Publish the Twist message and sleep 1 cycle      
            self.cmd_vel.publish(move_cmd)
            
            r.sleep()

        # Stop the robot before the rotation
        move_cmd = Twist()
        
        self.cmd_vel.publish(move_cmd)
        
        rospy.sleep(1)

    def get_odom(self, msg):
        if self.start_time is None:
            self.start_time = rospy.Time.now()
            
        self.position = msg.pose.pose.position 
        self.velocity = msg.twist.twist
        delta_velocity = sqrt(pow((self.velocity.linear.x - self.start_velocity.linear.x), 2) + 
                            pow((self.velocity.linear.y - self.start_velocity.linear.y), 2))
        acceleration = delta_velocity / (msg.header.stamp - self.start_time).to_sec()
        if acceleration > self.max_acceleration:
            self.max_acceleration = acceleration
            rospy.loginfo("Max acceleration: " + str(acceleration))
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    #try:
    TestAccleration()
    #except:
        #rospy.loginfo("Acceleration test terminated.")

