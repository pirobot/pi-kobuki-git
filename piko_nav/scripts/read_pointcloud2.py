#!/usr/bin/env python

"""
    follower2.py - Version 1.1 2013-12-20
    
    Follow a "person" by tracking the nearest object in x-y-z space.
    
    Relies on PCL ROS nodelets in the launch file to pre-filter the
    cloud on the x, y and z dimensions.
    
    Based on the follower application by Tony Pratkanis at:
    
    http://ros.org/wiki/turtlebot_follower
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

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
from roslib import message
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class ReadPointCloud():
    def __init__(self):
        rospy.init_node("read_point_cloud")
        
        rospy.loginfo("Subscribing to ply point cloud...")
        
        # Wait for the pointcloud topic to become available
        rospy.wait_for_message('ply_point_cloud', PointCloud2)

        # Subscribe to the point cloud
        self.pc_subscriber = rospy.Subscriber('ply_point_cloud', PointCloud2, self.process_point_cloud, queue_size=1)
        
    def process_point_cloud(self, msg):
        # Read in the x, y, z coordinates of all points in the cloud
        for point in point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            

        