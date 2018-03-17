#!/usr/bin/env python

""" follow_me.py - Version 1.1 2016-06-20

    Subscribe to PointStamped /target_topic to get a goal point and move the robot to with a given distance of the goal.

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
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import tf2_py
import tf
from tf.transformations import quaternion_from_euler
from math import atan2, cos, sin, sqrt, copysign, pow

class FollowMe():
    def __init__(self):
        rospy.init_node('follow_me', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        # How quickly should we update the tracker
        self.rate = rospy.Rate(rospy.get_param("~rate", 1))
        
        self.base_link = rospy.get_param("~base_link", "base_link")
        
        # Linear and angular threshold differences from last goal before updating goal
        self.linear_tracking_threshold = rospy.get_param("~linear_tracking_threshold", 0.2)
        self.angular_tracking_threshold = rospy.get_param("~angular_tracking_threshold", 0.05)

        # Minimum distance to keep between robot and target
        self.min_distance = rospy.get_param("~min_distance", 0.8)
        
        # Tolerance around min distance
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.1)
        
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        # Cancel any left over goals from previous run
        self.move_base.cancel_all_goals()
        
        # Initialize tf listener
        self.tf = tf.TransformListener()

        # Allow tf to catch up        
        rospy.sleep(2)
        
        self.last_target_odom = PointStamped()
        
        self.last_goal_sent = rospy.Time.now()
        
        # Wait for the target topic to become alive
        rospy.loginfo("Waiting for target topic...")
        
        rospy.wait_for_message('target_topic', PointStamped)
                
        # Subscribe to the goal_pose topic
        rospy.Subscriber('target_topic', PointStamped, self.follow_target, queue_size=1)
                        
        rospy.loginfo("Target messages detected. Starting tracker...")
            
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
        
    def follow_target(self, msg):
        lookup_time = msg.header.stamp
        success = False
        while not success:
            try:
                # Get the target location in the odom frame
                self.tf.waitForTransform('/odom', msg.header.frame_id, lookup_time, rospy.Duration(3.0))
                target_in_odom =  self.tf.transformPoint('/odom', msg)

                # Get the target location in the base frame
                self.tf.waitForTransform(self.base_link, msg.header.frame_id, lookup_time, rospy.Duration(3.0))
                target_in_base = self.tf.transformPoint(self.base_link, msg)
                
                success = True
            except (Exception, tf2_py.ExtrapolationException, tf2_py.ConnectivityException, tf2_py.LookupException):
                rospy.logerr("TF Exception!")
                rospy.sleep(0.1)
                        
        # x-y coordinates of target in odom frame
        x_odom = target_in_odom.point.x
        y_odom = target_in_odom.point.y
                
        # x-y coordinates of target in base frame
        x_base = target_in_base.point.x
        y_base = target_in_base.point.y
        
        # x-y coordinates of last target postion in odom frame
        last_target_odom_x = self.last_target_odom.point.x
        last_target_odom_y = self.last_target_odom.point.y
        
        # Distance between current target and last target in the odom frame
        distance_target_moved = sqrt(pow(x_odom - last_target_odom_x, 2) + pow(y_odom - last_target_odom_y, 2))
        
        # Distance between target and robot in the base frame
        distance_to_target = sqrt(x_base * x_base + y_base * y_base)

        # Compute the yaw angle to the target in the base frame
        yaw = atan2(y_base, x_base)
        q_array = quaternion_from_euler(0, 0, yaw)
        target_quaternion = Quaternion(q_array[0], q_array[1], q_array[2], q_array[3])
                        
        if (last_target_odom_x == 0 and last_target_odom_y == 0) or distance_target_moved > self.linear_tracking_threshold or abs(yaw) > self.angular_tracking_threshold:
            self.last_target_odom = target_in_odom
            goal_pose = Pose()
            goal_pose.position = target_in_base.point

            # Don't get closer than the minimum distance to the target
            if abs(distance_to_target - self.min_distance) > self.distance_tolerance:
                goal_distance = max(0, distance_to_target - self.min_distance)
                goal_pose.position.x = goal_distance * cos(yaw)
                goal_pose.position.y = goal_distance * sin(yaw)
        else:
            return

        # Set the orientation parallel to the vector between the robot and target
        goal_pose.orientation = target_quaternion
        
        # Zero out the z component of the goal
        goal_pose.position.z = 0.0
        
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = self.base_link
        goal.target_pose.pose = goal_pose
        goal.target_pose.header.stamp = rospy.Time.now()
                
        self.last_goal_sent = rospy.Time.now()
        
        self.move_base.send_goal(goal)
        
        self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        FollowMe()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follow_me node terminated.")
