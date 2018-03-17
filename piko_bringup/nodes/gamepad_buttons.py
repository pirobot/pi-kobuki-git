#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class GamepadButtons():
    def __init__(self):
        rospy.init_node("gamepad_buttons")
        
        self.button2command = {1: 'autodock',
                               2: '',
                               3: '',
                               4: '',
                               5: '',
                               5: '',
                               6: '',
                               7: '',
                               8: ''
                               }
        
        # Publish the location or command associated with the button pressed
        self.button_pub = rospy.Publisher('/gamepad_command', String, queue_size=1)
        
        # Subscribe to the /joy topic
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.pub_button_command)
        
        
    def pub_button_command(self, msg):
        buttons = msg.buttons
        
        pressed = [i for i,e in enumerate(buttons) if e == 1]
        
        try:
            button = pressed[0] + 1
        
            command = String()
             
            command.data = self.button2command[button]
            
            rospy.loginfo(command.data)
             
            self.button_pub.publish(command)
        except:
            pass


if __name__ == '__main__':
    GamepadButtons()
    rospy.spin()
    

