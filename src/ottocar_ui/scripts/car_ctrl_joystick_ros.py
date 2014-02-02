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
from sensor_msgs.msg import Joy

def time():
    return rospy.Time.now().to_sec()

class Accelerator(object):
    """docstring for Accelerator"""
    def __init__(self, minimum_speed, maximum_speed):
        super(Accelerator, self).__init__()
        
        self.speed = 0
        self.minimum_speed = minimum_speed
        self.maximum_speed = maximum_speed
        self.acceleration = 0

        self.last_time = time()

    def update(self):
        dtime = time()-self.last_time  # delta time in seconds
        self.speed += dtime * self.acceleration
        self.speed = min(max(self.speed,self.minimum_speed),self.maximum_speed)

        self.last_time = time()

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
        value = self.axis_rescale(value,-self.angle_min,0,0,self.angle_max)
        # value = self.axis_rescale(value,-128,0,0,127)
        # self.pub_angle.publish(Int8(data=int(value)))
        self.pub_angle.publish(Float32(data=(value)))
        print 'angle: ', value

    def accelerate_cmd(self, value):
        self.accelctrl.set_axis_value(value)

        speed = self.accelctrl.speed

        # motor does not really start when abs(speed) is below a certain value
        # so set it zero, that also improves the users ability to stop the car (i.e. speed=0) by accelerating/braking
        if abs(speed) < 6:
            speed = 0


        print 'speed: ', speed
        
        self.pub_speed.publish(Int8(data=int(speed)))
		

    def brake_to_zero(self):
        self.accelctrl.speed = 0
        self.accelctrl.acceleration = 0
        self.pub_rps.publish(Float32(data=self.accelctrl.speed))	

    def update(self):
        if self.joy==None:
            return

        if (self.joy.axes[2] == -1) and (self.joy.axes[5] == -1) and (self.joy.buttons[4] == 1) and (self.joy.buttons[4] == 1):
            self.run = False
            return

        if self.joy.buttons[3] == 1:
            self.pause = True
        if self.joy.buttons[1] == 1:
            self.pause = False

        if not self.pause:
            self.accelctrl.update()

            if self.joy.buttons[2] == 1:
                self.brake_to_zero()
            else:
                self.accelerate_cmd(self.joy.axes[1])
            self.angle_cmd(-self.joy.axes[3])
        

    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."

    def spin(self):
        r = rospy.Rate(20) # 10hz
        while(self.run):
            self.update()
            r.sleep()

        # rospy.spin() 

    def callback_joy(self,msg):
        self.joy = msg
        print self.joy
    
    def __init__(self):
        super(Node, self).__init__()

        self.run = True
        self.pause = False
        signal.signal(signal.SIGINT, self.keyboard_interupt)  



        rospy.init_node('joystick')

        rospy.Subscriber('/joy', Joy, self.callback_joy)

        # publisher for messages
        self.pub_angle = rospy.Publisher('yaw_rate_cmd', Float32, tcp_nodelay=True)
        self.pub_rps = rospy.Publisher('rps_cmd', Float32, tcp_nodelay=True)
        self.pub_speed = rospy.Publisher('speed_cmd', Int8, tcp_nodelay=True)

        self.joy = None

        self.angle_min = 3
        self.angle_max = 3
        # self.angle_min = 127
        # self.angle_max = 127
        # self.angle_min = 60
        # self.angle_max = 60
        #speed = 127 # warp 10
        speed = 40  # good speed 
        #speed = 15  # moderate 


        #accel = 10
        accel = 20
        #accel = 40
        # accel = 40000

#	accel = 1000

        # brake = accel * 10.0
        brake = accel * 2.0
        # brake = accel * 1.0
        # self.accel = Accelerator(max_acceleration = 1, max_speed=20)
        self.accelctrl = AccelerationCarControl(    #accelerations given in 1/s
            minimum_speed=-speed, maximum_speed=speed,
            max_forward_acceleration=accel, 
            max_forward_brake=brake, 
            max_reverse_acceleration=accel, 
            max_reverse_brake=brake)
        
        self.spin()




if __name__ == '__main__':
    Node()
    pygame.quit()
