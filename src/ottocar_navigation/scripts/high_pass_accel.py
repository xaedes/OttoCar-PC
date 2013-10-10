#!/usr/bin/env python

import pygame
from time import sleep
import signal

import math
import sys

from os.path import basename

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy


import numpy as np

from numpy import sin, linspace, pi, arange
from numpy.fft import fft,ifft
from pylab import plot, show, title, xlabel, ylabel, subplot
# from scipy import fft, arange

class MeasureSampleRate(object):
    """docstring for MeasureSampleRate"""
    def __init__(self, update_interval, gain):
        super(MeasureSampleRate, self).__init__()
        self.sample_rate = 1
        self.gain = gain
        self.update_interval = update_interval #in number of samples
        self.first_samplerate = False
        self.n_samples = 0
        self.last_time = rospy.Time.now().to_sec()

    def add_sample(self):
        self.n_samples = (self.n_samples + 1) % self.update_interval
        if self.n_samples == 0:
            self.update_sample_rate(self.update_interval)

    def update_sample_rate(self, n_new_samples = 1):
        dtime = rospy.Time.now().to_sec() - self.last_time
        self.last_time = rospy.Time.now().to_sec()

        current_sample_rate = n_new_samples / dtime

        if self.first_samplerate:
            self.first_samplerate = False
            self.sample_rate = current_sample_rate

        self.sample_rate = self.sample_rate * (1-self.gain) + current_sample_rate * self.gain        

    def __complex__(self):
        return complex(self.sample_rate)
    def __int__(self):
        return int(self.sample_rate)
    def __long__(self):
        return long(self.sample_rate)
    def __float__(self):
        return float(self.sample_rate)


class Window(object):
    """docstring for Window"""
    def __init__(self, window_size = 10, n_signals = 1):
        super(Window, self).__init__()
        
        self.window_size = window_size
        self.n_signals = n_signals
        self._buffer = np.zeros(shape=(self.window_size, self.n_signals))    # internal buffer where data is stored
        self.pointer = 0

        self.sample_rate = MeasureSampleRate(10,gain=0.5)


    def add_sample(self,signals):
        self.sample_rate.add_sample()

        if not self.is_full():
            self._buffer[self.pointer] = signals
            self.pointer += 1
        else:
            self._buffer = np.roll(self._buffer,-1)
            self._buffer[-1] = signals

    def is_full(self):
        return self.pointer == self._buffer.shape[0]

    # the last window_size items
    def window(self):
        return self._buffer[-self.window_size:,:]

    def set_window_size(self,new_window_size):
        if new_window_size > self._buffer.shape[0]:
            # increase buffer
            # print self._buffer.shape
            # print np.zeros(shape=(new_window_size - self._buffer.shape[0],self.n_signals)).shape
            self._buffer = np.vstack([self._buffer, np.zeros(shape=(new_window_size - self._buffer.shape[0],self.n_signals))])
            self.pointer = self._buffer.shape[0]

        self.window_size = new_window_size

class Filter(object):
    """docstring for Filter"""
    def __init__(self, window):
        super(Filter, self).__init__()

        self.window = window
        self.filtered = window.window()[:,:]

    def update(self):
        pass

    def get(self):
        return self.filtered[-1,:]

class HighPassSharpCutoff(Filter):
    """docstring for HighPass"""
    def __init__(self, window, cutoff):
        super(HighPassSharpCutoff, self).__init__(window)

        self.cutoff = cutoff

    def maximum_freq(self):
        return float(self.window.sample_rate) / 2

    def update(self):
        _fft = fft(self.window.window(),axis=0)
        freqs = np.fft.fftfreq(self.window.window_size, 1 / float(self.window.sample_rate) )
        # print np.real(_fft[:,0])
        _fft[abs(freqs) < self.cutoff,:] = 0
        # print abs(freqs) < self.cutoff
        # _fft[np.logical_and(freqs >= 0, freqs < self.cutoff),:] = 0
        # _fft[:,:] = 10
        # print np.logical_and((freqs >= 0), (freqs < self.cutoff))
        # print np.real(_fft[:,0])
        self.filtered = np.real(ifft(_fft,axis=0))
        

class Node(object):
    """Subscribes to imu data with orientation and removes gravity by applying a digital high-pass-filter to the acceleration."""
    # http://docs.scipy.org/doc/numpy/reference/routines.fft.html

    def __init__(self, ):
        super(Node, self).__init__()
        rospy.init_node(basename(__file__).replace('.','_'))
        signal.signal(signal.SIGINT, self.keyboard_interupt)

        if rospy.get_param('~use_joystick',False):
            rospy.Subscriber('/joy/', Joy, self.callback_joystick)

        rospy.Subscriber('/accelerometer/raw', Vector3, self.callback)
        self.pub = rospy.Publisher('/accelerometer/high_pass', Vector3)

        self.window_size = rospy.get_param('~window_size',10)

        self.window = Window(self.window_size, 3)
        self.filter = HighPassSharpCutoff(self.window,20)

        self.last_time = rospy.Time.now().to_sec()

        self.stop = False
        rospy.spin()



    def callback_joystick(self, joystick):
        dtime = rospy.Time.now().to_sec() - self.last_time

        # self.filter.cutoff = abs(joystick.axes[1]) * self.filter.maximum_freq()
        self.filter.cutoff = max(0.0,min(self.filter.maximum_freq(),self.filter.cutoff + (joystick.axes[1]) * dtime * 0.005))
        self.window_size = max(1.0,self.window_size + (-joystick.axes[3]) * dtime * 0.005)
        print 'cutoff:', self.filter.cutoff, 'window_size:', self.window_size, 'maximum_freq:', self.filter.maximum_freq()
        # print self.window_size
        self.window.set_window_size(int(self.window_size))

        self.last_joystick = joystick

    def callback(self, data):
        # print "sdlfz setou e 0z"
        if self.stop:
            return

        self.window.add_sample([data.x,data.y,data.z])

        # print self.window.pointer

        if self.window.is_full():
            # self.filter.cutoff = self.filter.maximum_freq() / 2
            self.filter.update()
            filtered = self.filter.get()
            self.pub.publish(Vector3(filtered[0],filtered[1],filtered[2]))

        
    def keyboard_interupt(self, signum, frame):
        rospy.signal_shutdown('keyboard interupt')
        sys.exit()


if __name__ == '__main__':
    Node()