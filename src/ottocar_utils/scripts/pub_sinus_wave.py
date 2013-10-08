#!/usr/bin/env python

import pygame
from time import sleep
import signal
import rospy
from std_msgs.msg import Float32
from os.path import basename

from math import sin, pi

        

class Node(object):
    """

    example usage: rosrun ottocar_utils pub_sinus_wave.py _freqs:=[0.1,10,25] _ampl:=[1,0.01,0.1] _sampling_rate:=100"""

    def __init__(self, ):
        super(Node, self).__init__()
        rospy.init_node(basename(__file__).replace('.','_'))
        signal.signal(signal.SIGINT, self.keyboard_interupt)  
        self.run = True



        self.frequencies = rospy.get_param('~freqs',[0.1])            # in hertz
        self.amplitudes = rospy.get_param('~ampl',[1])

        self.sampling_rate = rospy.get_param('~sampling_rate',100)    # in hertz
        self.sampling_interval =  1.0 / self.sampling_rate            # in seconds

        self.total_time = 0.0                                         # in seconds

        self.pub = rospy.Publisher('/sinus_wave', Float32)

        self.spin()

    def update(self):
        # dtime = self.clock.get_time() / 1000.0  # delta time in seconds
        # print dtime, self.sampling_interval
        # dtime = self.sampling_interval
        # dtime 

        # self.total_time += dtime
        self.total_time = rospy.Time.now().to_sec()

        value = sum([a * sin(self.total_time * f * 2 * pi) for (f,a) in zip(self.frequencies,self.amplitudes)])

        self.pub.publish(value)

    def callback_imu(self, data):
        self.pose.orientation = data.orientation
        


    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."


    def spin(self):
        self.clock = pygame.time.Clock()
        self.clock.tick()

        while(self.run):
            self.update()
            self.clock.tick(self.sampling_rate)

        # rospy.spin()

if __name__ == '__main__':
    Node()