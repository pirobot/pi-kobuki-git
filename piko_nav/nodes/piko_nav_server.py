#!/usr/bin/env python

"""
    piko_nav_server.py - Version 1.0 2016-09-10
    
    Move the robot to a specified location.
    
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
import actionlib
from actionlib import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from jr_utils.utilities import create_location_markers
from jr_msgs.msg import Location
from jr_msgs.srv import GotoLocation, GotoLocationResponse, GotoLocationRequest
from yaml import load
from math import sqrt

class NavServer():
    def __init__(self):
        rospy.init_node("piko_nav_server")

        # Set the shutdown function (stop the robot)
#        rospy.on_shutdown(self.shutdown)
        
        # Get the path of the file listing valid locations
        nav_config_file = rospy.get_param('~nav_config_file', 'config/locations.yaml')
        
        # Assume an average speed for the robot
        self.robot_ave_speed = rospy.get_param('~robot_ave_speed', 0.05)
        
        # Load the location data
        with open(nav_config_file, 'r') as config:
            self.locations = load(config)
        
        # Convert position/orientation pairs to ROS poses
        for location in self.locations:
            pose = PoseStamped()
            try:
                pose.header.frame_id = location['frame_id']
            except:
                pose.header.frame_id = '/map'
                
            pose.pose.position.x = location['position'][0]
            pose.pose.position.y = location['position'][1]
            pose.pose.position.z = location['position'][2]
            
            pose.pose.orientation.x = location['orientation'][0]
            pose.pose.orientation.y = location['orientation'][1]
            pose.pose.orientation.z = location['orientation'][2]
            pose.pose.orientation.w = location['orientation'][3]

            location['pose'] = pose
                        
        # Keep track of the robot's position as returned by AMCL
        self.robot_pose = PoseStamped()
            
        # Navigation goal state return values
        self.nav_goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                                'SUCCEEDED', 'ABORTED', 'REJECTED',
                                'PREEMPTING', 'RECALLING', 'RECALLED',
                                'LOST']
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait up to 5 seconds for the action server to become available
        if self.move_base.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Connected to move_base action server")
        else:
            rospy.logwarn("Timed out waiting for move_base action server")

        # Subscribe to the /amcl_pose topic so we know where the robot is located
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        
        rospy.loginfo("Waiting for AMCL...")
        
        try:
            rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, 5)
            while self.robot_pose == PoseStamped():
                rospy.sleep(0.1)
                rospy.loginfo("AMCL ready.")
        except:
            rospy.logwarn("Timed out waiting for AMCL")
        
        # Create some visualization markers for the various target locations
        location_marker_pub, location_markers = create_location_markers(locations=self.locations, marker_scale=0.2, show_labels=True)
        
        # Create a service to accept location requests
        rospy.Service('/goto_location', GotoLocation, self.GotoLocationHandler)
        
        rospy.loginfo("Ready to accept navigation requests.")
         
        # Run the tree
        while not rospy.is_shutdown():
            location_marker_pub.publish(location_markers)
            rospy.sleep(0.1)
            
    def GotoLocationHandler(self, req):
        sucess = False
                        
        if req.location.name is not None:
            [target_pose] = [location['pose'] for location in self.locations if location['name'] == req.location.name]
            rospy.loginfo("Going to location: " + str(req.location.name))

        elif req.location.pose != PoseStamped():
            target_pose = req.location.pose
                        
        # Intialize the goal
        goal = MoveBaseGoal()
        
        # Use the request frame_id to define goal frame_id
        goal.target_pose.header.frame_id = target_pose.header.frame_id
        
        # Set the time stamp to "now"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set the goal pose to request pose
        goal.target_pose.pose = target_pose.pose
        
        # Compute the timeout based on the distance to the goal
        robot_x = self.robot_pose.pose.position.x
        robot_y = self.robot_pose.pose.position.y
        
        goal_x = goal.target_pose.pose.position.x
        goal_y = goal.target_pose.pose.position.y

        distance_to_goal = sqrt((robot_x - goal_x) * (robot_x - goal_x) + (robot_y - goal_y) * (robot_y - goal_y))
        
        eta = 1.5 * distance_to_goal / self.robot_ave_speed + 60
        
        if eta > 60:
            rospy.loginfo("Allowing " + str(round(eta/60.0, 1)) + " minutes to go " + str(round(distance_to_goal, 1)) + " meters...")
        else:
            rospy.loginfo("Allowing " + str(round(eta, 0)) + " seconds to go " + str(round(distance_to_goal, 1)) + " meters...")

        # Start the robot moving toward the goal
        state = self.move(goal, eta)
        
        if state == GoalStatus.SUCCEEDED:
            success = True
        else:
            success = False
        
        # TODO: Add retry loop if failure

        return GotoLocationResponse(success)
    
    def move(self, goal, timeout):
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)
                
        # Allow timeout minutes to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(timeout)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out reaching goal")
                
        state = self.move_base.get_state()
        
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
        
        rospy.loginfo("Move base finished with status: " + str(self.nav_goal_states[state]))
        
        return state
    
    def amcl_callback(self, msg):
        self.robot_pose = msg.pose
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    NavServer()
