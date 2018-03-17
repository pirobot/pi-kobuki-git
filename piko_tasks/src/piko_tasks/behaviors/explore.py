#!/usr/bin/env python

""" explore.py - Version 1.0 2015-06-07

    Move the robot to explore the current environment.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2015 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
import actionlib
import networkx as nx
import matplotlib.pyplot as plot

class Room(object):
    def __init__(self, name, usedFor=None):
        self.name = name
        self.usedFor = usedFor

class Explore():
    def __init__(self):
        rospy.init_node("explore")
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        living_room = Room("Living Room", usedFor="relaxing")
        dining_room = Room("Dining Room", usedFor="eating")
        kitchen = Room("Kitchen", usedFor="cooking")
        bathroom = Room("Bathroom", usedFor="hygiene")
        bedroom = Room("Bedroom", usedFor="sleeping")
        hallway = Room("Hallway", usedFor="moving")
        foyer = Room("Foyer", usedFor="entering")

        house = nx.Graph()
        house.add_nodes_from([living_room, dining_room, kitchen, bathroom, bedroom, hallway, foyer])
        house.add_edge(living_room, dining_room, distance=4.6)
        house.add_edge(living_room, hallway, distance=5.9)
        house.add_edge(hallway, foyer, distance=4.9)
        house.add_edge(dining_room, kitchen, distance=2.9)
        house.add_edge(kitchen, hallway, distance=2.1)
        house.add_edge(hallway, bathroom, distance=3.6)
        house.add_edge(hallway, bedroom, distance=5.4)
        
        path = nx.shortest_path(house, dining_room, bedroom)
        
        for node in path:
            print node.name
        
        #print [room.name for room in house.nodes()]
        
        pos = nx.spring_layout(house)
        
        room_labels = dict()
        label_pos = dict()
        
        for node in house.nodes():
            room_labels[node] = node.name
            x, y = pos[node]
            label_pos[node] = (x, y+0.1)
                    
        nx.draw_networkx(house, pos, with_labels=False)
        nx.draw_networkx_labels(house, label_pos, room_labels)

        #nx.draw_graphviz(house, "neato")
        plot.show(block=False)
        
    def shutdown(self):
        rospy.loginfo("Shutting down explore node")
        plot.close()
        
if __name__ == '__main__':
    Explore()
    rospy.spin()

