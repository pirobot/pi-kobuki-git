#!/usr/bin/env python

"""
    piko_goto_location.py - Version 1.0 2016-09-10
    
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
from geometry_msgs.msg import Pose
from jr_msgs.msg import Location
from jr_msgs.srv import GotoLocationRequest, GotoLocationResponse, GotoLocation
from yaml import load

class GotoLocationNode():
    def __init__(self):
        rospy.init_node("goto_location_node")
        
        # Get the path of the configuration file
        nav_config_file = rospy.get_param('~nav_config_file', 'config/locations.yaml')
        
        requested_location = rospy.get_param('~requested_location', None)
        
        # Load the location data
        with open(nav_config_file, 'r') as config:
            self.locations = load(config)

        # Connect to the goto_location service
        rospy.wait_for_service('/goto_location', 60)

        goto_service = rospy.ServiceProxy('/goto_location', GotoLocation)
        
        request = GotoLocationRequest()
        request.location.header.stamp = rospy.Time.now()
        request.location.name = requested_location
        request.location.pose.header.frame_id = "map"
        request.location.pose.header.stamp = rospy.Time.now()
                
        response = goto_service(request)
                
        rospy.loginfo(response)


if __name__ == '__main__':
    GotoLocationNode()

