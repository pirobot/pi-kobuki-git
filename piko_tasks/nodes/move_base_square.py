#!/usr/bin/env python

""" move_base_square.py - Version 1.1 2013-12-20

    Command a robot to move in a square using move_base actions..

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        
        # How big is the square we want the robot to navigate?
        square_size = rospy.get_param("~square_size", 0.5) # meters
        
        # Create a list to hold the target quaternions (orientations)
        quaternions = list()
        
        # First define the corner orientations as Euler angles
        euler_angles = (pi/2, pi, 3*pi/2, 0)
        
        # Then convert the angles to quaternions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)
        
        # Create a list to hold the waypoint poses
        waypoints = list()
        
        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        #waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))
        waypoints.append(Pose(Point(-0.510, 1.509, 0.000), Quaternion(0, 0, 1, 0)))
        #
        waypoints.append(Pose(Point(0.504, 1.010, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707)))
        #waypoints.append(Pose(Point(self.square_size, self.square_size, 0.0), quaternions[1]))
        waypoints.append(Pose(Point(-1.350, 1.715, 0.000), Quaternion(0.000, 0.000, 0.997, 0.071)))
        waypoints.append(Pose(Point(0.0, square_size, 0.0), quaternions[2]))
        
        # Initialize the visualization markers for RViz
        self.init_markers()
        
        # Set a visualization marker at each waypoint        
        for waypoint in waypoints:           
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)
            
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        
        # Initialize a counter to track waypoints
        i = 0
        
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                            'SUCCEEDED', 'ABORTED', 'REJECTED',
                            'PREEMPTING', 'RECALLING', 'RECALLED',
                            'LOST']
        
        # Cycle through the four waypoints
        while i < 4 and not rospy.is_shutdown():
            # Update the marker display
            self.marker_pub.publish(self.markers)
            
            # Intialize the waypoint goal
            goal = MoveBaseGoal()
            
            # Use the map frame to define goal poses
            goal.target_pose.header.frame_id = 'map'
            
            # Set the time stamp to "now"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # Set the goal pose to the i-th waypoint
            goal.target_pose.pose = waypoints[i]
            
            # Start the robot moving toward the goal
            rospy.loginfo("Goal set to Waypoint " + str(i))
            self.move(goal)
            
            i += 1
        
    def move(self, goal):
            self.goal_status_reported = False
            self.action_finished = False
            
            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal, done_cb=self.done_cb)
            
            # Allow result_timeout seconds to get there
            result_timeout = 30
            time_so_far = 0.0
            action_finished = False
            finished_within_time = False
            
            tick = 1.0 / 20.0
            
            while True:
                rospy.sleep(tick)
                time_so_far += tick
                
                if self.action_finished:
                    state = self.move_base.get_state()
    
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Goal succeeded!")
                    else:
                        print "RESULT STATE:", self.goal_states[state]
                        
                    break
                        
                elif time_so_far > result_timeout:
                    rospy.loginfo("Timed out achieving goal")
                    break
                    
    def done_cb(self, status, result):
            # Check the final status
            self.goal_status = status
            self.action_finished = True
            
            if not self.goal_status_reported:
                rospy.loginfo("move_base ended with status " + str(self.goal_states[status]))
                self.goal_status_reported = True
                    
    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'odom'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
