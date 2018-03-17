#!/usr/bin/env python

""" move_base_tracker_test.py - Version 1.1 2016-06-14

    Send a series of move_base goals to whatever robot is listening.

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
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import atan2

class NavTest():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        # How quickly should we update the tracker
        rate = rospy.Rate(rospy.get_param("~rate", 1))
        
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        rospy.loginfo("Starting move_base tracker test")

        # Starting position for the robot
        start_x = 0.0
        start_y = 0.0
        
        # How far to move the target on each update
        delta_x = 0.1
        delta_y = 0.0
        
        # Compute the appropriate yaw angle
        theta = atan2(delta_y, delta_x)
        q_array = quaternion_from_euler(0, 0, theta)
        quat = Quaternion(q_array[0], q_array[1], q_array[2], q_array[3])

        start_pose = Pose(Point(start_x, start_y, 0.0), quat)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose = start_pose
        goal.target_pose.header.stamp = rospy.Time.now()
            
        rospy.loginfo("Going to initial pose")
        
        # The first goal does not seem to get published so do it twice
        self.move_base.send_goal(goal)
        rospy.sleep(1)
        self.move_base.send_goal(goal)

        self.move_base.wait_for_result(rospy.Duration(120))

        track_x = start_x
        track_y = start_y
        
        # Move the target delta_x and delta_y each loop
        while not rospy.is_shutdown():
            track_x += delta_x
            track_y += delta_y
            
            theta = atan2(track_y, track_x)
            q_array = quaternion_from_euler(0, 0, theta)
            quat = Quaternion(q_array[0], q_array[1], q_array[2], q_array[3])
            
            goal.target_pose.pose =  Pose(Point(track_x, track_y, 0.0), quat)
            goal.target_pose.header.stamp = rospy.Time.now()
            
            #rospy.loginfo("Going to: " + str(goal.target_pose.pose.position))
            
            self.move_base.send_goal(goal)
            
            rate.sleep()
            
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
