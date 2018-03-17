#!/usr/bin/env python

"""
    face_recognizer_to_pose.py - Version 1.0 2016-06-22
    
    Republish a cob_perception_msgs/DetectionArray message as a PoseStamped message
    
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
from geometry_msgs.msg import PoseStamped
from cob_perception_msgs.msg import DetectionArray
from tf.transformations import quaternion_from_euler
from math import sqrt

class PubPose():
    def __init__(self):
        rospy.init_node('head_position_to_pose')

        # Subscribe to the head_positions topic
        rospy.Subscriber('input_topic', DetectionArray, self.pub_pose_message)
        
        # The PoseStamped message pubtlisher
        self.pose_pub = rospy.Publisher('output_topic', PoseStamped, queue_size=5)
        
        rospy.loginfo("Re-publishing COB Face Positions message as PoseStamped message")
        
    def pub_pose_message(self, msg):
        min_distance = 10000
        target_head = None
        
        # Pick the closest detection
        for head in msg.detections:
            pos = head.pose.pose.position
            distance = sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z)
            if distance < min_distance:
                target_head = head
                min_distance = distance
        
        if target_head is None:
            return
        
        # Extract the pose value
        target_pose = target_head.pose
                
        # Set an arbitrary orientation so we can view the pose in RViz
        orientation = quaternion_from_euler(0, -1.57, 0)
        
        # Add the orientation to the target pose
        target_pose.pose.orientation.x = orientation[0]
        target_pose.pose.orientation.y = orientation[1]
        target_pose.pose.orientation.z = orientation[2]
        target_pose.pose.orientation.w = orientation[3]
        
        # Publish the pose
        self.pose_pub.publish(target_pose)
 
if __name__ == '__main__':
    try:
        target = PubPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
