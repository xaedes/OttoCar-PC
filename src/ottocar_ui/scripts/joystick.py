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

class Accelerator(object):
    """docstring for Accelerator"""
    def __init__(self, minimum_speed, maximum_speed):
        super(Accelerator, self).__init__()
        
        self.speed = 0
        self.minimum_speed = minimum_speed
        self.maximum_speed = maximum_speed
        self.acceleration = 0

        self.clock = pygame.time.Clock()
        self.clock.tick()

    def update(self):
        dtime = self.clock.get_time() / 1000.0  # delta time in seconds

        self.speed += dtime * self.acceleration
        self.speed = min(max(self.speed,self.minimum_speed),self.maximum_speed)

        self.clock.tick()

class AccelerationCarControl(Accelerator):
    """docstring for AccelerationCarControl"""
    def __init__(self, minimum_speed, maximum_speed, max_forward_acceleration, max_forward_brake, max_reverse_acceleration, max_reverse_brake):
        super(AccelerationCarControl, self).__init__(minimum_speed, maximum_speed)

        self.max_forward_acceleration = max_forward_acceleration
        self.max_forward_brake = max_forward_brake
        self.max_reverse_acceleration = max_reverse_acceleration
        self.max_reverse_brake = max_reverse_brake
        
    def set_axis_value(self, value):
        if(self.speed < 0):
            self.acceleration = Node.axis_rescale(value,-self.max_reverse_acceleration,0,0,self.max_reverse_brake)
        else:
            self.acceleration = Node.axis_rescale(value,-self.max_forward_brake,0,0,self.max_forward_acceleration)


class Node(object):
    """Handles input from joystick to publish /speed_cmd and /angle_cmd.

    Remarks:
    Joystick may not be exactly calibrated, thus it may not be possible to reach maximum values calculated from joystick axes.
    """

    @staticmethod
    def rescale(value,to_min,to_max,from_min=-1,from_max=1):
        return ((value-from_min) / (from_max-from_min)) * (to_max-to_min) + to_min

    @staticmethod
    def axis_rescale(value,to_absmax_negative,to_absmin_negative,to_absmin_positive,to_absmax_positive):
        '''Rescales input from axis.

        Axis input is in [-1,1]. 
        Negative values are mapped from [-1,0] to [to_absmax_negative,to_absmin_negative].
        Positive values are mapped from [0,+1] to [to_absmin_positive,to_absmax_positive].
        '''

        if(value < 0):
            return Node.rescale(value,to_absmin_negative,to_absmax_negative,from_min = 0,from_max = -1)
        else:
            return Node.rescale(value,to_absmin_positive,to_absmax_positive,from_min = 0,from_max = 1)

    def angle_cmd(self, value):
        # value = self.rescale(value,270,450)
        print value
        value = self.axis_rescale(value,270,375,375,450)
        self.pub_angle.publish(Int16(data=int(value)))
        print 'angle: ', value

    def speed_cmd(self, value):
        print value
        value = self.axis_rescale(value,-20,0,10,50)      
        # value = self.rescale(value,-20,20)      # moderate speed
        # value = self.rescale(value,-127,127)  # maximum speed
        self.pub_speed.publish(Int8(data=int(value)))
        print 'speed: ', value

    def accelerate_cmd(self, value):
        self.accelctrl.set_axis_value(value)
        self.accelctrl.update()
        print 'speed: ', self.accelctrl.speed

        
        # self.pub_speed.publish(Int8(data=int(10)))
        self.pub_speed.publish(Int8(data=int(self.accelctrl.speed)))


    def update(self):
        self.accelerate_cmd( -self.joystick.get_axis( 1 ))
        # self.speed_cmd( -self.joystick.get_axis( 1 ))
        self.angle_cmd( self.joystick.get_axis( 3 ))
        

    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."

    def spin(self):
        while(self.run):
            # pygame events must be processed to get current values for joystick (happens in the background)
            pygame.event.pump()


            self.update()
            sleep(0.1)      

        # rospy.spin() 

    def assert_joystick_capabilities(self, i):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        # must have at least axis 1 and 3, i.e. 4 axes

        if joystick.get_numaxes() < 4: 
            return False

        return True

    def init_joystick(self):
        pygame.init()
        pygame.joystick.init()
        print "Number of joysticks: ", pygame.joystick.get_count()
        if pygame.joystick.get_count()==0 :
            print "No joystick plugged in. "
            print "Plug in joystick and start again"
            return False


        print "Looking for capable joystick"

        for i in xrange(0,pygame.joystick.get_count()):
            if self.assert_joystick_capabilities(i):
                self.joystick = pygame.joystick.Joystick(i)
                self.joystick.init()
                print "Capable joystick found."
                return True

        print "No capable joystick found."
        print "Plug in capable (see self.assert_joystick_capabilities) joystick and start again"

        return False
    
    def __init__(self):
        super(Node, self).__init__()

        self.run = True
        signal.signal(signal.SIGINT, self.keyboard_interupt)  



        # Initialize the joysticks
        if self.init_joystick():
            rospy.init_node('example')

            # publisher for messages
            self.pub_angle = rospy.Publisher('angle_cmd', Int16)
            self.pub_speed = rospy.Publisher('speed_cmd', Int8)

            # self.accel = Accelerator(max_acceleration = 1, max_speed=20)
            self.accelctrl = AccelerationCarControl(    #accelerations given in 1/s
                minimum_speed=-20, maximum_speed=20,
                max_forward_acceleration=10, 
                max_forward_brake=20, 
                max_reverse_acceleration=10, 
                max_reverse_brake=20)
            
            self.spin()




if __name__ == '__main__':
    Node()
    pygame.quit()
