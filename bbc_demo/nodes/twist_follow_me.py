#!/usr/bin/env python

"""
    twist_follow_me.py - Version 2.0 2013-08-23
    
    Move the base to track an object published on the /target PointStamped topic.
        
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
from geometry_msgs.msg import PointStamped, Point, Pose, PoseStamped, Twist
from math import copysign
import tf
import os, thread

# Initialize the node
rospy.init_node("twist_follow_me.py")
    
class PersonTracker():
    def __init__(self):
        
        rospy.on_shutdown(self.shutdown)
        
        self.rate = rospy.get_param('~rate', 10)
                
        self.tick = 1.0 / self.rate
        
        self.max_target_distance = rospy.get_param('~max_target_distance', 2.0)
            
        self.enable_target_search = rospy.get_param('~enable_target_search', False)
        
        # How long do we continue tracking at same velocity after target is lost?
        self.lost_detection_timeout = rospy.get_param('~lost_detection_timeout', 1.5)
        
        # How long we are willing to wait (in seconds) before searching?
        self.search_target_timeout = rospy.get_param('~search_target_timeout', 10)
        
        # How long we are willing to wait (in seconds) before searching?
        self.search_rotation_pause_time = rospy.get_param('~search_rotation_pause_time', 5)
        
        # What is the name of the camera link?
        self.camera_link = rospy.get_param('~camera_link', 'camera_link')

        # Default base rotation and linear speeds
        self.default_angular_speed = rospy.get_param('~default_angular_speed', 0.5)
        self.default_linear_speed = rospy.get_param('~default_linear_speed', 0.1)

        # Maximum linear and angular speeds
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.0)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)
        
        # Minimum linear and angular speeds
        self.min_angular_speed = rospy.get_param('~min_angular_speed', 1.0)
        self.min_linear_speed = rospy.get_param('~min_linear_speed', 0.05)
        
        # Desired approach distance to target
        self.approach_distance = rospy.get_param('~approach_distance', 1.0)
        
        # How far off from center does the head have to be before we react
        self.angular_threshold = rospy.get_param('~angular_threshold', 0.1)
        
        # How far away from the target distance does the target have to be before we react
        self.linear_threshold = rospy.get_param('~linear_threshold', 0.1)
        
        # The angular and linear gain parameters determine how responsive the base movements are.
        # If these are set too high, oscillation can result.
        self.angular_gain = rospy.get_param('~angular_gain', 1.0)
        self.linear_gain = rospy.get_param('~linear_gain', 1.0)
        
        # Initialize tf listener
        self.tf = tf.TransformListener()

        # Allow tf to catch up        
        rospy.sleep(2)
        
        # Set a flag to indicate when the target has been lost
        self.target_visible = False
        
        # Set a timer to determine how long a target is no longer visible
        self.target_lost_time = 0.0
        
        # A flag to indicate whether we're in wait mode
        self.searching = False
        
        # Initialize the base speeds to 0
        self.angular_speed = 0.0
        self.linear_speed = 0.0
        
        # Get a lock for updating the self.move_cmd values
        self.lock = thread.allocate_lock()
        
        # Publisher for moving the robot
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        
        # Initialize the twist command
        self.cmd_vel = Twist()
        
        # Subscribe to the target topic for tracking people
        rospy.Subscriber('target_topic', PointStamped, self.update_base_motion, queue_size=1)
        
        rospy.loginfo("Starting follower.")
                        
        while not rospy.is_shutdown():
            # Acquire the lock
            #self.lock.acquire()
            
            try:
                # If we have lost the target, stop tracking and set a timer
                if not self.target_visible:
                    if not self.searching:
                        # Keep track of how long the target is lost
                        self.target_lost_time += self.tick

                        if self.target_lost_time > self.lost_detection_timeout:
                            # Stop robot smoothly
                            self.stop_robot_smoothly(linear=True, angular=True)

                            self.cmd_vel.angular.z = 0.0
                            rospy.loginfo("No target visible... " + str(self.target_lost_time))
                            
                        # If the target is lost long enough, search for it
                        if self.enable_target_search and self.target_lost_time > self.search_target_timeout:
                            rospy.loginfo("Searching for target...")
                            self.searching = True
                            self.search_for_target()
                else:
                    rospy.loginfo("Target detected.")
                    self.target_visible = False
                    self.searching = False
                    self.target_lost_time = 0.0
                    
                    self.cmd_vel.linear.x = self.linear_speed
                    self.cmd_vel.angular.z = self.angular_speed   
                    
                # Send the motion command to the base
                self.cmd_vel_pub.publish(self.cmd_vel)
                    
            finally:
#                 # Release the lock
#                 self.lock.release()
                pass
                                    
            rospy.sleep(self.tick)
            
    def stop_robot_smoothly(self, linear=False, angular=False):
        if linear:
            if self.cmd_vel.linear.x < 0.05:
                self.cmd_vel.linear.x = 0.0
            else:
                self.cmd_vel.linear.x *= 0.2
        
        if angular:
            if self.cmd_vel.angular.z < 0.5:
                self.cmd_vel.angular.z = 0.0
            else:
                self.cmd_vel.angular.z *= 0.2
        
    def update_base_motion(self, msg):
        # Acquire the lock
        #self.lock.acquire()
        
        try:
            # If message is empty, return immediately
            if msg == PointStamped():
                return
    
            # Get position component of the message
            target = PointStamped()
            target.header.frame_id = msg.header.frame_id
            target.point = msg.point
                                
            # Project the target point onto the camera link
            camera_target = self.tf.transformPoint(self.camera_link, target)
            
            # The virtual camera image is in the y-z plane
            pan = -camera_target.point.y
            tilt = -camera_target.point.z
            
            # Compute the distance to the target in the x direction
            distance = float(abs(camera_target.point.x))
            
            # Only respond to targets closer than the max distance
            if distance > self.max_target_distance:
                return
            
            # If we get this far, the target is visible
            self.target_visible = True
            
            # Convert the pan and tilt values from meters to radians
            try:
                pan /= distance
                tilt /= distance
            except:
                # Check for exceptions (NaNs) and use minumum range as fallback
                pan /= 0.5
                tilt /= 0.5
                
            # Rotate the robot only if the angular displacement of the target point exceeds the threshold 
            if abs(pan) > self.angular_threshold:
                # Set the angular speed proportional to the horizontal displacement of the target
                self.angular_speed = -copysign(max(self.min_angular_speed, min(self.max_angular_speed, abs(self.angular_gain * pan))), pan)
            else:
                self.angular_speed = 0.0
            
            # Move the robot forward/backward only if the distance from target exceeds threshold
            delta_distance = distance - self.approach_distance
            
            if abs(delta_distance) > self.linear_threshold:
                # Set the linear speed proportional to the offset from the approach distance
                self.linear_speed = copysign(max(self.min_linear_speed, min(self.max_linear_speed,  abs(self.linear_gain * delta_distance))), delta_distance)
            else:
                self.linear_speed = 0.0
                
        finally:
            # Release the lock
            #self.lock.release()
            pass
                        
    def search_for_target(self):
        # Field of view of Kinect or Primesense cameras is about 1 radian
        fov = 1.0
        
        angle_rotated = 0.0
                
        # Rotate slowly and pause every FOV/2 radians to check for detections
        while not self.target_visible and not rospy.is_shutdown():
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = self.default_angular_speed
            self.cmd_vel_pub.publish(self.cmd_vel)
            rospy.sleep(self.tick)
            
            angle_rotated += self.tick * self.default_angular_speed
            
            if abs(angle_rotated) > fov:
                angle_rotated = 0.0
                pause_time = 0.0
                while not self.target_visible and not rospy.is_shutdown():
                    rospy.sleep(self.tick)
                    pause_time += self.tick
                    if pause_time > self.search_rotation_pause_time:
                        break
        
    def shutdown(self):
        rospy.loginfo("Shutting down person tracking node...")
        self.cmd_vel_pub.publish(Twist())
                   
if __name__ == '__main__':
    try:
        PersonTracker()
    except rospy.ROSInterruptException:
        rospy.loginfo("Person tracking node terminated.")




