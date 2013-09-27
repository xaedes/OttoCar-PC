#!/usr/bin/env python

import pygame
from time import sleep
import signal

import rospy
import os
import sys

from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import Float32


class Node(object):
    """docstring for Node"""

    def rescale(self, value,to_min,to_max,from_min=-1,from_max=1):
        return ((value-from_min) / (from_max-from_min)) * (to_max-to_min) + to_min

    def angle_cmd(self, value):
        value = self.rescale(value,270,450)
        self.pub_angle.publish(Int16(data=int(value)))
        print 'angle: ', value
        # pass

    def speed_cmd(self, value):
        value = self.rescale(value,-20,20)
        # value = self.rescale(value,-127,127)
        self.pub_speed.publish(Int8(data=int(value)))
        print 'speed: ', value

    def update(self):
        self.speed_cmd( -self.joystick.get_axis( 1 ))
        self.angle_cmd( self.joystick.get_axis( 3 ))
        

    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."

    def spin(self):
        self.run = True
        signal.signal(signal.SIGINT, self.keyboard_interupt)    
        while(self.run):
            # pygame events must be processed to get current values for joystick (happens in the background)
            pygame.event.pump()


            self.update()
            sleep(0.1)      

        # rospy.spin() 

    def __init__(self):
        super(Node, self).__init__()
        rospy.init_node('example')

        # publisher for messages
        self.pub_angle = rospy.Publisher('angle_cmd', Int16)
        self.pub_speed = rospy.Publisher('speed_cmd', Int8)

        # # Initialize the joysticks
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()


        # rospy.Subscriber('/current', Int16, self.callback_current)

        self.spin()




if __name__ == '__main__':
    Node()
    pygame.quit()

# # -------- Main Program Loop -----------
# while done==False:
#     # EVENT PROCESSING STEP
#     for event in pygame.event.get(): # User did something
#         if event.type == pygame.QUIT: # If user clicked close
#             done=True # Flag that we are done so we exit this loop
        
            
#         joystick = pygame.joystick.Joystick(0)
#         joystick.init()
    
#         obj.speed_cmd( -joystick.get_axis( 1 ))
#         obj.kp_cmd( joystick.get_axis( 3 ))


    
#     # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
    
#     # Go ahead and update the screen with what we've drawn.
#     pygame.display.flip()

#     # Limit to 20 frames per second
#     clock.tick(20)
    
# # Close the window and quit.
# # If you forget this line, the program will 'hang'
# # on exit if running from IDLE.
# pygame.quit ()