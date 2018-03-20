#!/usr/bin/env python

import rospy
import functools
import py_trees
import py_trees_ros
import py_trees.console as console
from py_trees.common import Status
from py_trees.blackboard import Blackboard
import sys

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from arbotix_msgs.srv import SetSpeed, Relax, Enable
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

blackboard = Blackboard()

class MoveHead(py_trees_ros.actions.ActionClient):
    def __init__(self, name="Action Client", action_spec=FollowJointTrajectoryAction, action_goal=None, action_namespace="/head_controller/follow_joint_trajectory",
                 override_feedback_message_on_running="moving head", angle_goal=[0.0, 0.0], *args, **kwargs):    
        super(MoveHead, self).__init__(name, *args, **kwargs)
        
        self.action_spec = action_spec
        self.action_namespace = action_namespace
        self.override_feedback_message_on_running = override_feedback_message_on_running
        
        head_joints = ['head_pan_joint', 'head_tilt_joint']
        duration = 3.0
        
        trajectory = JointTrajectory()
        trajectory.joint_names = head_joints
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = angle_goal
        trajectory.points[0].velocities = [0.0 for i in head_joints]
        trajectory.points[0].accelerations = [0.0 for i in head_joints]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
            
        # Send the trajectory to the head action server
        rospy.loginfo('Moving the head to goal position...')
        
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = trajectory
        self.goal.goal_time_tolerance = rospy.Duration(0.0)
            
    def initialise(self):
        rospy.loginfo('Moving head to [pan=%.2f, tilt=%.2f] radians', self.goal.trajectory.points[0].positions[0], self.goal.trajectory.points[0].positions[0])
        self.action_goal = self.goal
        
#     def terminate(self, new_status=Status.INVALID):
#         rospy.logwarn("TERMINATED!")
                    
class RelaxServos(py_trees.Behaviour):
    def __init__(self, name):
        self.servos_relaxed = False
        
        super(RelaxServos, self).__init__(name)
        
    def update(self):
        try:
            for joint in blackboard.joints:
                blackboard.relax_servo[joint]()

                #blackboard.servo_speed[joint](blackboard.default_joint_speed)
                #blackboard.enable_servo[joint](False)
            rospy.sleep(1)
            return Status.SUCCESS
        except:
            return Status.FAULURE
    
class CheckServosReady(py_trees.Behaviour):
    def __init__(self, name, **kwargs):
        super(CheckServosReady, self).__init__(name, **kwargs)
        
    def update(self):
        if blackboard.servos_initialized:
            return Status.SUCCESS
        else:
            return Status.FAILURE

class InitServos(py_trees.Behaviour):
    def __init__(self, name):
        blackboard.servos_initialized = False
        
        super(InitServos, self).__init__(name)
        
    def setup(self, default_joint_speed=0.5, timeout=15):
        # The list of joints is stored in the /arbotix/joints parameter
        blackboard.joints = rospy.get_param('/arbotix/joints', '')
 
        blackboard.default_joint_speed = default_joint_speed
        blackboard.servos_initialized = False
         
        self.init_servos()
        
        blackboard.servos_initialized = True
        
        return True
 
    def update(self):
        if blackboard.servos_initialized:
            return Status.SUCCESS
        else:
            return Status.RUNNING
         
    def init_servos(self):
        blackboard.servo_speed = dict()
        blackboard.servo_position = dict()
        blackboard.enable_servo = dict()
        blackboard.relax_servo = dict()
        blackboard.joint_states = dict()
        
        # Subscribe to the joint_states topic
        rospy.Subscriber('joint_states', JointState, self.get_joint_states)
 
        # Connect to the set_speed services and define a position publisher for each servo
        rospy.loginfo("Waiting for joint controllers services...")
                 
        for joint in blackboard.joints:
            # The position controllers
            blackboard.servo_position[joint] = rospy.Publisher('/' + joint + '/command', Float64, queue_size=5)
            
            # A service to enable (disable) a servo
            enable_service = '/' + joint + '/enable'
            rospy.wait_for_service(enable_service) 
            blackboard.enable_servo[joint] = rospy.ServiceProxy(enable_service, Enable)
             
            # Start the servo in the enabled state
            blackboard.enable_servo[joint](True)
            
            # The set_speed service
            set_speed_service = '/' + joint + '/set_speed'
            rospy.wait_for_service(set_speed_service)
            blackboard.servo_speed[joint] = rospy.ServiceProxy(set_speed_service, SetSpeed, persistent=True)
 
            # Initialize the servo speed to the default_joint_speed
            blackboard.servo_speed[joint](blackboard.default_joint_speed)
            
            # A service to relax a servo (disable torque)
            relax_service = '/' + joint + '/relax'
            rospy.wait_for_service(relax_service) 
            blackboard.relax_servo[joint] = rospy.ServiceProxy(relax_service, Relax)
             
            # Start the servo in the relaxed state
            blackboard.relax_servo[joint]()
            
        rospy.loginfo("Servos initialized")
        
    def get_joint_states(self, msg):
        for joint in msg.name:
            blackboard.joint_states[joint] = msg.position[msg.name.index(joint)]

        