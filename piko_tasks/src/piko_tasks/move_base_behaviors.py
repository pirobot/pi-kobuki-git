#!/usr/bin/env python

import rospy
import functools
import py_trees
import py_trees_ros
import py_trees.console as console
from py_trees.common import Status
from py_trees.blackboard import Blackboard
import move_base_msgs.msg as move_base_msgs
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose
import actionlib_msgs.msg as actionlib_msgs
from tf.transformations import quaternion_from_euler
import sys
from math import radians

blackboard = Blackboard()

class MoveBaseSimple(py_trees_ros.actions.ActionClient):
    def __init__(self, name="MoveBase Client", action_spec=MoveBaseAction, action_goal=None, action_namespace="/move_base",
                 override_feedback_message_on_running="moving robot", simple_goal=[0.0, 0.0], base_link="base_link", *args, **kwargs):    
        super(MoveBaseSimple, self).__init__(name, *args, **kwargs)
        
        self.action_spec = action_spec
        self.action_namespace = action_namespace
        self.override_feedback_message_on_running = override_feedback_message_on_running

        self.simple_goal = simple_goal
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = base_link
                
        self.counter = 0
        
        rospy.loginfo("Connection to move_base server...")

    def initialise(self):
        rospy.loginfo('Action set to move the robot to [%.2f meters and %.2f degrees]', self.simple_goal[0], self.simple_goal[1])
        
#         if self.counter > 0:
#             self.simple_goal[0] += 0.1
          
        self.goal.target_pose.pose = self.make_move_base_goal_pose(self.simple_goal)
        
        self.action_goal = self.goal
          
        #self.counter += 1
       
    def make_move_base_goal_pose(self, simple_goal):
        pose = Pose()
        
        pose.position.x = simple_goal[0]
 
        quaternion_angle = quaternion_from_euler(0, 0, radians(simple_goal[1]))
 
        pose.orientation.x = quaternion_angle[0]
        pose.orientation.y = quaternion_angle[1]
        pose.orientation.z = quaternion_angle[2]
        pose.orientation.w = quaternion_angle[3]
        
        return pose