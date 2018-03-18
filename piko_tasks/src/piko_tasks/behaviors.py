#!/usr/bin/env python

import rospy
import functools
import py_trees
import py_trees_ros
import py_trees.console as console
from py_trees.common import Status
from py_trees.blackboard import Blackboard
from jr1_msgs.srv import SayPhrase, SayPhraseResponse
from sound_play.libsoundplay import SoundClient
import sensor_msgs
import sys

blackboard = Blackboard()

class SayPhrase(py_trees.Behaviour):
    def __init__(self, name, script_path, phrase=None):        
        super(SayPhrase, self).__init__(name)
        
        self.phrase = phrase
        
        # Set the wave file path if used
        self.wavepath = rospy.get_param("~wavepath", script_path + "/../config/wave_files")
        
        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        
        # Create the sound client object
        self.soundhandle = SoundClient(blocking=True)
        
        # Wait a moment to let the client connect to the sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        
    def setup(self, timeout=15):
        # Announce that we are ready
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        rospy.sleep(1)
        self.soundhandle.say("Ready", self.voice)
        rospy.sleep(2)
        
        return True
    
    def update(self):
        try:
            self.soundhandle.say(self.phrase, self.voice)
            return Status.SUCCESS
        except:
            return Status.FAILURE


