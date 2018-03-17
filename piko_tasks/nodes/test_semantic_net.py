#!/usr/bin/env python

"""
    test_semantic_net.py - Version 1.0 2015-06-06
    
    Test semantic network functions.
    
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
from rdflib import Graph, BNode, Namespace

ex = Namespace("http://ex.com/schema#")

hasARTag = ex.hasARTag

living_room = BNode('living room')
dining_room = BNode('dining room')
hallway = BNode('hallway')

g = Graph()

g.add((flea, smaller, mouse))
g.add((mouse, smaller, elephant))

print "Smaller than elephant:"
for i in g.transitive_objects(smaller, elephant):
    print i

    