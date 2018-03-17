#!/usr/bin/env python

""" test_angular_accleration.py - Version 1.1 2016-8-17

    Rotate the robot 360 degrees at the target speed and measure angular acceleration.

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
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import tf
from math import radians, copysign, pi
import PyKDL

class TestAngularAcceleration():
    def __init__(self):
        # Give the node a name
        rospy.init_node('test_angular_acceleration', anonymous=False)
        
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        
        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 50)
        r = rospy.Rate(self.rate)
        
        # The test angle is 360 degrees
        self.test_angle = radians(rospy.get_param('~test_angle', 360.0))

        self.speed = rospy.get_param('~speed', 1.) # radians per second
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # The base frame is usually base_link or base_footprint
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
        
        # Start the clock
        start_time = rospy.Time.now()
        
        # Track max acceleration
        max_acceleration = 0
        
        # Get the current rotation angle from tf
        self.odom_angle = self.get_odom_angle()
        
        last_angle = self.odom_angle
        turn_angle = 0
        error = self.test_angle - turn_angle
        
        while self.test_angle - turn_angle > 0:
            if rospy.is_shutdown():
                return
            
            # Rotate the robot to reduce the error
            move_cmd = Twist()
            move_cmd.angular.z = copysign(self.speed, error)
            self.cmd_vel.publish(move_cmd)
            r.sleep()
         
            # Get the current rotation angle from tf                   
            self.odom_angle = self.get_odom_angle()
            
            # Compute how far we have gone since the last measurement
            delta_angle = self.normalize_angle(self.odom_angle - last_angle)
            
            # Add to our total angle so far
            turn_angle += delta_angle
            
            # Compute acceleration to this point
            acceleration = abs(delta_angle) / (rospy.Time.now() - start_time).to_sec()
            
            if acceleration > max_acceleration:
                max_acceleration = acceleration
                rospy.loginfo("Max acceleration: " + str(acceleration))

            # Compute the new error
            error = self.test_angle - turn_angle

            # Store the current angle for the next comparison
            last_angle = self.odom_angle
                            
        # Stop the robot
        self.cmd_vel.publish(Twist())
        
    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        
        # Convert the rotation from a quaternion to an Euler angle
        return self.quat_to_angle(Quaternion(*rot))

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]
            
    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    #try:
    TestAngularAcceleration()
    #except:
        #rospy.loginfo("Calibration terminated.")
