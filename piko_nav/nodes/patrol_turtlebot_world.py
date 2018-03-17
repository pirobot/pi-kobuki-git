#!/usr/bin/env python

"""
    patrol_turtlebot_world.py - Version 1.0 2016-09-10
    
    Patrol the Gazebo Turtlebot world using named locations.
    
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

class Patrol():
    def __init__(self):
        rospy.init_node("patrol_turtlebot_world")

        # Connect to the goto_location service
        rospy.wait_for_service('/goto_location', 60)

        goto_service = rospy.ServiceProxy('/goto_location', GotoLocation)
        
        locations = ["home", "box", "dumpster", "barrier", "bookshelf", "water_tower"]
        
        request = GotoLocationRequest()
        request.location.pose.header.frame_id = "map"
      
        while not rospy.is_shutdown():
            for location in locations:
                request.location.header.stamp = rospy.Time.now()
                request.location.name = location
                request.location.pose.header.stamp = rospy.Time.now()                        
                response = goto_service(request)
                rospy.loginfo(response)


if __name__ == '__main__':
    Patrol()

