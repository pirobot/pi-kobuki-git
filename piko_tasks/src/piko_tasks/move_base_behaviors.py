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
from tf.transformations import quaternion_from_euler
import sys

blackboard = Blackboard()

class MoveBaseSimple(py_trees_ros.actions.ActionClient):
    def __init__(self, name="MoveBase Client", action_spec=MoveBaseAction, action_goal=None, action_namespace="/move_base",
                 override_feedback_message_on_running="moving base", simple_goal=[0.0, 0.0], base_link='base_link', *args, **kwargs):    
        super(MoveBaseSimple, self).__init__(name, *args, **kwargs)
        
        self.action_spec = action_spec
        self.action_namespace = action_namespace
        self.override_feedback_message_on_running = override_feedback_message_on_running
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = base_link
        goal.target_pose.pose.position.x = simple_goal[0]
        
        quaternion_angle = quaternion_from_euler(0, 0, simple_goal[1])

        goal.target_pose.pose.orientation.x = quaternion_angle[0]
        goal.target_pose.pose.orientation.y = quaternion_angle[1]
        goal.target_pose.pose.orientation.z = quaternion_angle[2]
        goal.target_pose.pose.orientation.w = quaternion_angle[3]
            
        # Send the goal to the move_base action server
        rospy.loginfo('Moving the robot to [%.2f, %.2f]', simple_goal[0], simple_goal[1])
    
        self.action_goal = goal
    
    def terminate(self, new_status=Status.SUCCESS):
        pass
