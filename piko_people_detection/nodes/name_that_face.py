#!/usr/bin/env python

"""
    name_that_face.py - Version 1.0 2013-11-10
    
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
from cob_perception_msgs.msg import *
from sound_play.libsoundplay import SoundClient
import sys

class NameThatFace():
    def __init__(self):
        rospy.init_node('name_that_face')
        
        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_cmu_us_clb_arctic_clunits")
        
        # Set the wave file path if used
        self.wavepath = rospy.get_param("~wavepath", None) + "/../sounds"
        
        self.use_sound = rospy.get_param("~use_sound", True)
        
        if self.use_sound:
            # Create the sound client object
            self.soundhandle = SoundClient()
            
            # Wait a moment to let the client connect to the sound_play server
            rospy.sleep(1)
            
            # Make sure any lingering sound_play processes are stopped.
            self.soundhandle.stopAll()
            
            # Announce that we are ready for input
            self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            rospy.sleep(1)
            
            self.soundhandle.say("Ready", self.voice)
   
        rospy.Subscriber('face_positions', DetectionArray, self.get_faces)
        
        self.last_message = None
                
    def get_faces(self, msg):
        message = ""
        
        for detection in msg.detections:
            left_right = detection.pose.pose.position.x

            message += detection.label

            if abs(left_right) < 0.3:
                message += " is straight ahead "
            elif left_right < 0:
                message += " is on my right "
            else:
                message += " is on my left "
                            
        if message == "":
            message = "Where is everyone?"
                
        if message != self.last_message:
            self.last_message = message

            rospy.loginfo(message)
            
            if self.use_sound:
                self.soundhandle.say(message, self.voice)
            
  
if __name__ == '__main__':
    try:
        NameThatFace()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Name-that-face node terminated.")
