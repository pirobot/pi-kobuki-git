#!/usr/bin/env python

"""
    cob_head_position_to_point.py - Version 1.0 2016-06-22
    
    Republish a cob_perception_msgs/DetectionArray message as a PointStamped message
    
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
from geometry_msgs.msg import PointStamped, PoseStamped
from cob_perception_msgs.msg import DetectionArray
from math import sqrt

class Head2Point():
    def __init__(self):
        rospy.init_node('head_position_to_point')

        # Subscribe to the head_positions topic
        rospy.Subscriber('input_topic', DetectionArray, self.pub_pose_message)
        
        # The PointStamped message pubtlisher
        self.point_pub = rospy.Publisher('output_topic', PointStamped, queue_size=5)
        
        rospy.loginfo("Re-publishing COB head position as PointStamped message")
        
    def pub_pose_message(self, msg):
        min_distance = 10000
        target_head = PoseStamped()
        
        # Pick the closest detection
        for head in msg.detections:
            pos = head.pose.pose.position
            distance = sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z)
            if distance < min_distance:
                target_head = head
                min_distance = distance
        
        if target_head == PoseStamped():
            return
        
        # Extract the pose value
        target_point = PointStamped()
        target_point.header = target_head.header
        target_point.point = target_head.pose.position
        
        # Publish the pose
        self.point_pub.publish(target_point)
 
if __name__ == '__main__':
    try:
        target = Head2Point()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
