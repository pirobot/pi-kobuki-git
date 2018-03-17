#!/usr/bin/env python

"""
    approach_ar_tag.py - AR tag identification and tracking - Version 1.0 2015-06-03
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2015 Patrick Goebel.  All rights reserved.

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

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Quaternion, Point
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from arbotix_msgs.srv import *
from std_msgs.msg import Float64

#from srrc_msgs.srv import DetectTarget, DetectTargetResponse
#from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker

from math import radians, degrees, pi, copysign, sin, cos
import PyKDL
import tf
import os
#from srrc_msgs.srv._DetectTarget import DetectTargetRequest

class State():
    SCANNING = 0
    HEAD_TRACKING = 1
    ROTATING = 2
    MOVING = 3
    STOPPED = 4
    TARGET_DETECTED = 5
    SCAN_FAILED = 6
    DOCKING_BACK = 7
    DOCKING_LEFT = 8
    DOCKING_RIGHT = 9
    DOCKING_FRONT = 10
    DOCKING_STARTED = 11
    DOCKING_DOCKED = 12

class ApproachTag():
    def __init__(self):
        rospy.init_node("approach_ar_tag")

        # Stop the robot and center the servos when shutting downs
        rospy.on_shutdown(self.shutdown)
        
        # Initialize parameters, subscribers, publishers, etc
        self.init_params()
        
        # Center the head camera servos
        self.center_head_servos()

        # Initialize the state of the robot
        self.state = State.STOPPED
        
        self.docking_state = State.DOCKING_FRONT

        ''' MAIN STEPS:
            1. Turn on ar_track_alvar detection using webcam
            2. Verify detection of first fiducial (attached below large marker)
            3. Run a dead reckoning movement pattern to get around to the other side of the
            home platform.
                (a) Turn left 90 degrees
                (b) Move forward X meters
                (c) Turn right 90 degrees
                (d) Move forward Y meters
                (e) Turn right 90 degrees
                (f) Move forward X meters
                (g) Turn right 90 degrees
                
        '''
        
        while not rospy.is_shutdown() and self.docking_state is None:
            rospy.sleep(0.1)     

    def scan_for_target(self, detection_result):
        rospy.loginfo("Scanning for target...")
                
        # Scan at the default speed
        self.servo_speed[self.head_pan_joint](self.default_joint_speed)          

        # Get the current pan position
        current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
        
        # Start the scan
        if not self.state == State.SCANNING:
            self.state = State.SCANNING
            
            # Get the horizontal and vertical FOV from the latest snapshot
            fov_x = detection_result.fov_x
            
            # Get the vertical FOV from the latest snapshot
            fov_y = detection_result.fov_y
            
            # Set the total number of scans needed to complete both fields of view
            self.max_scans = 1 + int(2.0 * (self.max_tilt - self.min_tilt) / fov_y)
            
            # Set the pan increment to 1/2 the FOV
            self.scan_pan_increment = fov_x / 2.0
            
            # Set the tilt increment to 1/2 the FOV
            self.scan_tilt_increment = fov_y / 2.0
            
            # Start the scan in the direction we are already looking
            if current_pan < 0:
                self.scan_pan_increment *= -1
  
        # Pan 1/2 of the FOV
        new_pan = current_pan + self.scan_pan_increment
        
        # Keep the rotation between the min and max settings
        if self.scan_pan_increment < 0:
            new_pan = max(-self.pan_limit, new_pan)
        else:
            new_pan = min(self.pan_limit, new_pan)
            
        self.pan_finished = False
        
        # Pan to the new angle
        while abs(current_pan - new_pan) > 0.05:
            self.servo_position[self.head_pan_joint].publish(new_pan)
            rospy.sleep(0.1)
            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
        
        # When we have reached max or min pan, reverse direction
        if abs(new_pan) == self.pan_limit:
            self.scan_pan_increment *= -1
            
            # Keep track of the number of scans so we can adjust tilt on each scan
            self.scan_count += 0.5
            
            if self.scan_count != 0.5:
                self.pan_finished = True
            
        # Adjust the tilt angle after each full pan, starting with the camera pointed up
        if self.use_camera_tilt and self.scan_count != 0 and self.pan_finished:
            
            self.servo_speed[self.head_tilt_joint](self.default_joint_speed)
            
            # Get the current tilt position
            current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
            
            # Start with the camera tilted up
            if self.scan_count == 1:
                if (current_tilt - self.min_tilt) < 0.05:
                    new_tilt = self.min_tilt + self.scan_tilt_increment
                else:
                    new_tilt = self.min_tilt
            else:                   
                # Set new tilt
                new_tilt = current_tilt + self.scan_tilt_increment
                
                # Keep the tilt between the max and min limits
                if self.scan_tilt_increment < 0:
                    new_tilt = max(self.min_tilt, new_tilt)
                else:
                    new_tilt = min(self.max_tilt, new_tilt)

            self.servo_position[self.head_tilt_joint].publish(new_tilt)

            # When we have reached max or min pan, reverse direction   
            if new_tilt == self.max_tilt or new_tilt == self.min_tilt:
                 self.scan_tilt_increment *= -1
                 
        if self.scan_count == self.max_scans:
            self.state = State.SCAN_FAILED
            rospy.loginfo("SCANNING FOR TARGET FAILED!")
                
    def align_camera(self, delta_pan, delta_tilt):
        # Make sure the servo speed is set to the default
        self.servo_speed[self.head_pan_joint](self.default_joint_speed)
        
        try:
            # Get the current pan position
            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
            
            # Update the pan position
            pan = current_pan + delta_pan

            while abs(current_pan - pan) > self.pan_threshold:
                self.servo_position[self.head_pan_joint].publish(pan)
                rospy.sleep(0.2)
                current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]     
            
            # And the tilt position
            if self.use_camera_tilt:
                self.servo_speed[self.head_tilt_joint](self.default_joint_speed)
                current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
                
                tilt = current_tilt + delta_tilt
                
                while abs(current_tilt - tilt) > self.tilt_threshold:
                    self.servo_position[self.head_tilt_joint].publish(tilt)
                    rospy.sleep(0.2)
                    current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
                
        except:
            print "EXCEPTION trying to update pan/tilt position"
                   
    def move_robot_forward(self, distance):
        self.state = State.MOVING

        # Create the move_base goal
        goal = MoveBaseGoal()
            
        # Use the base frame to define goal poses relative to the robot itself
        goal.target_pose.header.frame_id = 'base_footprint'
            
        # Set the time stamp to "now"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Move the target distance
        goal.target_pose.pose.position.x = distance
        goal.target_pose.pose.orientation.w = 1.0
        
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal, feedback_cb=self.move_base_feedback_cb)
                                
        # Allow 2 minutes to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(120)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
       
        # Stop the robot manually just to be sure
        self.cmd_vel.publish(Twist())
          
    def align_robot_with_target(self):
        # Get the current head pan position
        current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
                
        # Get the current base rotation angle from tf
        (position, self.odom_angle) = self.get_odom()
        
        # If there is a TF exception from get_odom(), it returns None for position and odom_angle
        if position is None:
            return
        
        last_angle = self.odom_angle
        turn_angle = 0
                
        # Rotate the camera back to center
        self.servo_position[self.head_pan_joint].publish(0)
                      
        while abs(turn_angle - current_pan) > self.base_angular_tolerance:
            if rospy.is_shutdown():
                return
            
            move_cmd = Twist()
            move_cmd.angular.z = copysign(self.max_angular_speed, (current_pan - turn_angle))
            self.cmd_vel.publish(move_cmd)
            
            rospy.sleep(0.1)
         
            # Get the current rotation angle from tf                   
            (position, self.odom_angle) = self.get_odom()
            
            # Compute how far we have gone since the last measurement
            delta_angle = self.odom_angular_scale_correction * self.normalize_angle(self.odom_angle - last_angle)
            
            # Add to our total angle so far
            turn_angle += delta_angle
            last_angle = self.odom_angle
        
        # Stop the robot
        self.cmd_vel.publish(Twist())
        
    def rotate_robot(self, angle):
        # Get the current base rotation angle from tf
        (position, self.odom_angle) = self.get_odom()
        
        # If there is a TF exception from get_odom(), it returns None for position and odom_angle
        if position is None:
            return
        
        last_angle = self.odom_angle
        turn_angle = 0
                      
        while abs(turn_angle - angle) > self.base_angular_tolerance:
            if rospy.is_shutdown():
                return
            
            move_cmd = Twist()
            move_cmd.angular.z = copysign(self.max_angular_speed, (angle - turn_angle))
            self.cmd_vel.publish(move_cmd)
            
            rospy.sleep(0.1)
         
            # Get the current rotation angle from tf                   
            (position, self.odom_angle) = self.get_odom()
            
            # Compute how far we have gone since the last measurement
            delta_angle = self.odom_angular_scale_correction * self.normalize_angle(self.odom_angle - last_angle)
            
            # Add to our total angle so far
            turn_angle += delta_angle
            last_angle = self.odom_angle
        
        # Stop the robot
        self.cmd_vel.publish(Twist())
        
    def initiate_docking(self):
        if self.docking_state != State.DOCKING_STARTED:
            self.dock_state = State.DOCKING_STARTED
             
            # Rotate left 90 degrees
            self.rotate_robot(pi/2)
            
            # Move forward to clear the platform
            self.move_robot_forward(self.docking_clear_platform_distance)
                
            # Rotate right 90 degrees
            self.rotate_robot(pi/2)
            
            # Move forward to get behind the platform
            self.move_robot_forward(self.docking_move_behind_platform_distance)
            
            # Rotate right 90 degrees
            self.rotate_robot(pi/2)
            
            # Finished docking initiation
            self.docking_state = State.DOCKING_FRONT
            
            return
        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return (None, None)

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))
    
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
        
    def center_head_servos(self):
        rospy.loginfo("Centering servos.")

        self.servo_speed[self.head_pan_joint](self.default_joint_speed)
        current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]

        if self.use_camera_tilt:
            self.servo_speed[self.head_tilt_joint](self.default_joint_speed)
            current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]

        while abs(current_pan) > 0.05:
            self.servo_position[self.head_pan_joint].publish(0)
           
            rospy.sleep(0.1)
            
            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]

        if self.use_camera_tilt:
            while abs(current_tilt) > 0.05 :
                self.servo_position[self.head_tilt_joint].publish(0)
                
                rospy.sleep(0.1)
            
                current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
                
    def init_params(self):
        # Is the robot in the paused state at startup?
        self.robot_paused = False
        
        # What AR tag IDs are valid for the home base
        self.ar_tag_ids = rospy.get_param('~ar_tag_ids', [0])
        
        # What is the mapping between AR tags and sides of the home base?
        self.ar_tag_mapping = rospy.get_param('~ar_tag_mapping', None)
        
        # Do we use move_base or simple odometry-based motion?
        self.use_move_base = rospy.get_param("~use_move_base", True)
        
        # What are the names of the pan and tilt joints in the list of dynamixels?
        self.head_pan_joint = rospy.get_param('~head_pan_joint', 'head_pan_joint')
        self.use_camera_tilt = rospy.get_param('~use_camera_tilt', False)

        # Pan/tilt the camera to scan for target if it is not visible?
        self.scan_camera = rospy.get_param('~scan_camera', False)
        
        # Set the joint variable names
        if self.use_camera_tilt:
            self.head_tilt_joint = rospy.get_param('~head_tilt_joint', 'head_tilt_joint')
            self.joints = [self.head_pan_joint, self.head_tilt_joint]
        else:
            self.joints = [self.head_pan_joint]
        
        # Joint speeds are given in radians per second
        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.3)
        self.max_joint_speed = rospy.get_param('~max_joint_speed', 0.5)
        
        # The initial tilt angle of the camera in degrees
        self.initial_tilt_angle = radians(rospy.get_param('~initial_tilt_angle', 10))
        
        # The pan/tilt thresholds in degrees indicate what percentage of the image window
        # the ROI needs to be off-center before we make a movement
        self.pan_threshold = radians(rospy.get_param("~pan_threshold", 3.0))   # degrees
        self.tilt_threshold = radians(rospy.get_param("~tilt_threshold", 3.0)) # degrees
        
        # Set limits on the pan and tilt angles in degrees
        self.pan_limit = radians(rospy.get_param("~pan_limit", 90))  # degrees
        self.max_tilt = radians(rospy.get_param("~max_tilt", 60))    # degrees
        self.min_tilt = radians(rospy.get_param("~min_tilt", 0))     # degrees
        
        # Parameters controlling motion of the base
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.5) # radians per second
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.2)   # meters per second

        # Angular and linear tolerances for aligning with and moving toward the target
        self.base_angular_tolerance = rospy.get_param('~base_angular_tolerance', 0.05) # radians: 0.1 radian = 5.7 dgrees
        self.base_linear_tolerance = rospy.get_param('~base_linear_tolerance', 0.5)   # meters

        # Initialize the servo services and publishers
        self.servos_detected = False
        self.init_servos()
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # The base frame is usually base_footprint or base_link
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Wait a bit for the odom and base frames
        try:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(2.0))
        except:
            rospy.loginfo("Base node does not appear to be running.")
        
        if self.use_move_base:
            # Subscribe to the move_base action server
            self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            
            rospy.loginfo("Waiting for move_base action server...")
            
            # Wait 2 seconds for the action server to become available
            self.move_base.wait_for_server(rospy.Duration(2))

        # Subscribe the ar_pose_marker topic used by ar_track_alvar to publish marker poses
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.process_ar_tags)
        
        rospy.loginfo("Approach sequence started!")
        
        self.align_camera_attempts = 0
        self.align_robot_attempts = 0
        self.scan_count = 0
        self.docking_started = False
        self.ar_tag_detected = False
        
    def process_ar_tags(self, msg):
        # If the robot is paused, bail from the loop
        if self.robot_paused:
            rospy.sleep(1)
            return
                
        # Check for no markers detected
        if len(msg.markers) == 0:
            self.ar_tag_detected = False
        else:
            # Iterate through any tags detected
            for tag in msg.markers:
                # Skip any tags that are not in our list
                if not tag.id in self.ar_tag_ids:
                    continue
                #elif self.ar_tag_mapping[tag.id] == "front":
                elif tag.id == 1:                
                    self.docking_state = State.DOCKING_FRONT
                    self.ar_tag_detected = True
                    ar_front_tag = tag
        
        # If no tag is detected, scan for one
        if not self.ar_tag_detected:
            self.servo_speed[self.head_pan_joint](self.default_joint_speed)
            
            rospy.loginfo("SCANNING FOR AR TAG")
            
            # Start by panning the camera toward the left
            self.scan_for_ar_tag_pan_goal = self.pan_limit
            self.servo_position[self.head_pan_joint].publish(self.scan_for_ar_tag_pan_goal)
            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
            
            if abs(current_pan - self.scan_for_ar_tag_pan_goal) < self.pan_threshold:
                self.servo_position[self.head_pan_joint].publish(self.scan_for_ar_tag_pan_goal)
            else:
                # Pan the other way
                self.scan_for_ar_tag_pan_goal *= -1
        
        else:
            # Zero the motion by defaul
            cmd_vel = Twist()
            
            # Vector in on the front fiducial
            if abs(ar_front_tag.pose.pose.position.y) > self.base_angular_tolerance:
                rospy.loginfo("ALIGNING ROBOT WITH TARGET")
                cmd_vel.angular.z = copysign(0.5, ar_front_tag.pose.pose.position.y)
            if abs(ar_front_tag.pose.pose.position.x) > self.base_linear_tolerance:
                rospy.loginfo("MOVING TOWARD TARGET")
                cmd_vel.linear.x = copysign(0.1, ar_front_tag.pose.pose.position.x)
            else:
                cmd_vel = Twist()
                
            rospy.loginfo(cmd_vel)

            # Move or stop the robot
            self.cmd_vel.publish(cmd_vel)

    def init_servos(self):
        # Create dictionaries to hold the speed, position and torque controllers
        self.servo_speed = dict()
        self.servo_position = dict()
        self.relax_servo = dict()
        self.enable_servo = dict()
        
        # Monitor the joint states of the pan and tilt servos
        rospy.loginfo("Attempting to detect servos...")
        try:
            rospy.wait_for_message('joint_states', JointState, timeout=2)
        except:
            rospy.loginfo("Servos not detected.")
            return
        
        self.servos_detected = True

        self.joint_state = JointState()
        rospy.Subscriber('joint_states', JointState, self.update_joint_state, queue_size=5)

        # Wait until we actually have joint state values
        while self.joint_state == JointState():
            rospy.sleep(0.1)

        # Connect to the set_speed services and define a position publisher for each servo
        rospy.loginfo("Waiting for servo controllers services...")
                
        for joint in sorted(self.joints):
            # The set_speed services
            set_speed_service = '/' + joint + '/set_speed'
            rospy.wait_for_service(set_speed_service)
            self.servo_speed[joint] = rospy.ServiceProxy(set_speed_service, SetSpeed, persistent=False)

            # Initialize the servo speed to the default_joint_speed
            self.servo_speed[joint](self.default_joint_speed)

            # The position controllers
            self.servo_position[joint] = rospy.Publisher('/' + joint + '/command', Float64, queue_size=5)

            # A service to relax (disable torque) a servo     
            relax_service = '/' + joint + '/relax'
            rospy.wait_for_service(relax_service) 
            self.relax_servo[joint] = rospy.ServiceProxy(relax_service, Relax)
            
             # A service to enable/disable a servo     
            enable_service = '/' + joint + '/enable'
            rospy.wait_for_service(enable_service) 
            self.enable_servo[joint] = rospy.ServiceProxy(enable_service, Enable)
                       
            # Start the servo in the relaxed state
            self.relax_servo[joint]()
            
        rospy.loginfo("Connected to servo controllers.")
       
    def update_joint_state(self, msg):
        # Skip wheel joint states
        try:
            test = msg.name.index(self.head_pan_joint)
            self.joint_state = msg
        except:
            pass
        
    def move_base_feedback_cb(self, msg):
        pass
#         if not self.target_visible:
#             self.move_base.cancel_all_goals()
#             self.cmd_vel.publish(Twist())
#             self.state = State.STOPPED
        
    def shutdown(self):
        rospy.loginfo("Shutting down docking node.")
        
        self.center_head_servos()
    
        # Relax all servos to give them a rest.
        rospy.loginfo("Relaxing pan and tilt servos.")
    
        for servo in self.joints:
            self.relax_servo[servo]()
            
        os._exit(0)        
                   
if __name__ == '__main__':
    try:
        ApproachTag()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Home Base Docking node terminated.")
