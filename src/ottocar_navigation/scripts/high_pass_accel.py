#!/usr/bin/env python

import pygame
from time import sleep
import signal

import math
import sys

from os.path import basename

import rospy
from geometry_msgs.msg import Vector3


import numpy as np

from numpy import sin, linspace, pi, arange
from numpy.fft import fft,ifft
from pylab import plot, show, title, xlabel, ylabel, subplot
# from scipy import fft, arange

class Node(object):
    """Subscribes to imu data with orientation and removes gravity by applying a digital high-pass-filter to the acceleration."""
    # http://docs.scipy.org/doc/numpy/reference/routines.fft.html

    def __init__(self, ):
        super(Node, self).__init__()
        rospy.init_node(basename(__file__).replace('.','_'))
        signal.signal(signal.SIGINT, self.keyboard_interupt)  

        rospy.Subscriber('/accelerometer/raw', Vector3, self.callback)
        self.pub = rospy.Publisher('/accelerometer/high_pass', Vector3)

        self.window_size = rospy.get_param('~window_size',100)
        self.buffer = np.zeros(shape=(self.window_size,3),dtype=np.float64)
        self.pointer = 0

        self.last_time = rospy.Time.now().to_sec()
        self.sample_rate = 0
        self.sample_rate_gain = 0.5

        self.first_samplerate = True
        self.fill_buffer = True

        self.stop = False
        rospy.spin()

    def plotSpectrum(self,y,Fs):
        """
        Plots a Single-Sided Amplitude Spectrum of y(t)
        """

        n = len(y) # length of the signal
        k = arange(n)
        T = n/Fs
        frq = k/T # two sides frequency range
        frq = frq[range(n/2)] # one side frequency range

        Y = fft(y)/n # fft computing and normalization
        Y = Y[range(n/2)]

        print zip(frq,abs(Y))
        # print "yo"

        # plot(frq,abs(Y),'r') # plotting the spectrum
        # xlabel('Freq (Hz)')
        # ylabel('|Y(freq)|')

    def update_sample_rate(self):
        dtime = rospy.Time.now().to_sec() - self.last_time
        self.last_time = rospy.Time.now().to_sec()
        current_sample_rate = self.window_size / dtime

        if self.first_samplerate:
            self.first_samplerate = False
            self.sample_rate = current_sample_rate

        self.sample_rate = self.sample_rate * (1-self.sample_rate_gain) + current_sample_rate * self.sample_rate_gain

        # print self.sample_rate


    def callback(self, data):
        # print "sdlfz setou e 0z"
        if self.stop:
            return

        if self.fill_buffer:
            self.buffer[self.pointer,0] = data.x
            self.buffer[self.pointer,1] = data.y
            self.buffer[self.pointer,2] = data.z
            self.pointer = self.pointer + 1

            if self.pointer == self.window_size-1:
                self.fill_buffer = False
                self.update_sample_rate()

        else:
            self.buffer = np.roll(self.buffer,-1,axis=0)
            self.buffer[-1,0] = data.x
            self.buffer[-1,1] = data.y
            self.buffer[-1,2] = data.z

            Y = fft(self.buffer[:,0]) # fft computing ; normalization can by done by dividing through window_size
            # print Y.shape
            Y[0:(self.window_size/2)/2] = 0
            foo = ifft(Y)
            # print foo.shape

            self.pub.publish(Vector3(foo[-1],self.buffer[-1,1],self.buffer[-1,2]))
            # Y = Y[range(self.window_size/2)]



            # self.stop = True
            # self.plotSpectrum(list(self.buffer[:,0]),self.sample_rate)
            # self.stop = False
            
        # self.pub_imu.publish(self.imu_data)
        
    def keyboard_interupt(self, signum, frame):
        rospy.signal_shutdown('keyboard interupt')
        sys.exit()


if __name__ == '__main__':
    Node()