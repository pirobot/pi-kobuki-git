#!/usr/bin/env python

"""
    test_tree.py - Version 1.0 2013-03-18
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

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
from pi_trees_ros.pi_trees_ros import *
from piko_tasks.task_setup import *
import time

# A class to track global variables
class BlackBoard():
    def __init__(self):
        self.count1 = 1
        self.count2 = 0

# Initialize the black board
blackboard = BlackBoard()

class TestTree():
    def __init__(self):
        rospy.init_node("patrol_tree")
                
        # Create the root node
        BEHAVE = Sequence("BEHAVE", abort_children=True)
        
        TEST_SELECTOR = Selector("TEST_SELECTOR")
        
        COUNT1 = Count("Count1", 1, 3, 1, blackboard)
        
        COUNT2 = Count("Count2", 3, 1, -1, blackboard)

        
        LIMIT_MESSAGE = Limit("LIMIT_MESSAGE", max_executions=3)
        
        #LOOP_MESSAGE = Loop("LOOP_MESSAGE", iterations=3)
        
        MESSAGE1 = DisplayMessage("MESSAGE1", "Greetings Humans")
        
        MESSAGE2 = DisplayMessage("MESSAGE2", "See you later!")
        
        TEST_SEQUENCE = Sequence("TEST_SEQUENCE", [COUNT1, MESSAGE1, COUNT2, MESSAGE2], reset_after=False)


        GOODBYE = CallbackTask("GOODBYE", cb=self.display_message, cb_args=[1, "Goodbye!"])
        
        ALWAYS_FAIL = CallbackTask("ALWAYS_FAIL", cb=self.always_fail)
                
        LOOP_MESSAGE = loop(MESSAGE1)
        
        INVERT = TaskNot("INVERT")
        
        INVERT.add_child(ALWAYS_FAIL)
        
        UNTIL_FAIL = UntilFail("UNTIL_FAIL")
        
        UNTIL_FAIL.add_child(COUNT1)

        # Add the top level subtrees to the root node in order of priority
        TEST_SELECTOR.add_child(invert(ALWAYS_FAIL))
        TEST_SELECTOR.add_child(LOOP_MESSAGE)
        
        TEST_PARALLEL = ParallelAll("PARALLEL_ALL")
        
        TEST_PARALLEL.add_child(COUNT1)
        TEST_PARALLEL.add_child(MESSAGE1)
        
        #BEHAVE.add_child(until_fail(COUNT1))
        
        BEHAVE.add_child(TEST_SEQUENCE)
                
        # Display the tree before beginning execution
        print "Test Behavior Tree"
        print_tree(BEHAVE)
        
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(1)
            
    def display_message(self, count, message):
        for i in range(count):
            print i, message
            time.sleep(0.5)
        return True
    
    def always_fail(self):
        print "Always Fail"
        return Fail
    
class DisplayMessage(Task):
    def __init__(self, name, message):
        super(DisplayMessage, self).__init__(name)
        
        self.name = name
        self.message = message
 
    def run(self):
        if self.status != TaskStatus.SUCCESS:
            rospy.loginfo(self.message)
            self.status = TaskStatus.SUCCESS
                
        return self.status
    
    def reset(self):
        self.status = None
            
# A counting task that extends the base Task task
class Count(Task):
    def __init__(self, name, start, stop, step, blackboard, *args, **kwargs):
        super(Count, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.start = start
        self.stop = stop
        self.step = step
        self.count = start
        self.blackboard = blackboard
        
        print "Creating task Count", self.start, self.stop, self.step
 
    def run(self):
        if abs(self.count - self.stop - self.step) <= 0:
            self.status = TaskStatus.SUCCESS
        else:            
            time.sleep(0.5)
            
            print self.name, self.count
            
            self.count += self.step
            
            if self.name == "Count1":
                self.blackboard.count1 = self.count
                
            elif self.name == "Count2":
                self.blackboard.count2 = self.count
            
            if abs(self.count - self.stop - self.step) <= 0:
                self.status = TaskStatus.SUCCESS
            else:
                self.status = TaskStatus.RUNNING
                
        return self.status
            
    def abort(self):
        print "BB Count 1:", self.blackboard.count1
        print "Aborting", self, "!"
        self.reset()
    
    def reset(self):
        self.count = self.start
        if self.name == "Count1":
            self.blackboard.count1 = self.count
            
        elif self.name == "Count2":
            self.blackboard.count2 = self.count
            

if __name__ == '__main__':
    tree = TestTree()

