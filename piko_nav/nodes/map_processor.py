#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

class MapProcessor():
    def __init__(self):
        # Set a name for the node
        self.node_name = "map_processor"
        
        # Initialize the node
        rospy.init_node(self.node_name)
                
        # Set a shutdown function to clean up when terminating the node
        rospy.on_shutdown(self.shutdown)
        
        self.map = OccupancyGrid()
        
        # Subscribe to a map topic and set the callback function
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        rospy.loginfo('Map processor started')
        
        rospy.spin()
        
    def map_callback(self, msg):
        self.map = msg
        rospy.loginfo("Map width: %d", self.map.info.width)
        rospy.loginfo("Map height: %d", self.map.info.height)
        rospy.loginfo("Map resolution: %f", self.map.info.resolution)
        
        map_array = self.map.data
        
        rospy.loginfo("Size of map data array is: %d", len(map_array))

    
    def shutdown(self):
        rospy.loginfo('Shutting down ' + str(self.node_name))
        
if __name__ == '__main__':
    MapProcessor()
