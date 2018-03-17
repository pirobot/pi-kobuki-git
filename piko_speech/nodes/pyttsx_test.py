#!/usr/bin/env python

import pyttsx

engine = pyttsx.init()
volume = engine.getProperty('volume')

engine.setProperty('volume', volume-0.25)
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[2].id)

engine.say('Sally sells seashells by the seashore.')
#engine.say( 'The quick brown fox jumped over the lazy dog.' )
engine.runAndWait()
