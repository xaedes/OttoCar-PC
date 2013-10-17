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

from ottocar_utils import spark
from ottocar_utils import MeasureSampleRate

import numpy as np

from numpy import sin, linspace, pi, arange
from numpy.fft import fft,ifft
from pylab import plot, show, title, xlabel, ylabel, subplot
# from scipy import fft, arange




class Window(object):
    """docstring for Window"""
    def __init__(self, window_size = 10, n_signals = 1):
        super(Window, self).__init__()
        
        self.window_size = window_size
        self.n_signals = n_signals      # TODO rename signals to channels
        self._buffer = np.zeros(shape=(self.window_size, self.n_signals))    # internal buffer where data is stored
        self.pointer = 0

        self.sample_rate = MeasureSampleRate(10,gain=0.5)


    def add_sample(self,signals):
        self.sample_rate.add_sample()

        if not self.is_full():
            self._buffer[self.pointer] = signals
            self.pointer += 1
        else:
            self._buffer = np.roll(self._buffer,-1,axis=0)
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
            self.pointer = self._buffer.shape[0] - 1

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
        self.sparks = ""

    def maximum_freq(self):
        return float(self.window.sample_rate) / 2

    def freqs(self, shifted = False):
        freqs = np.fft.fftfreq(self.window.window_size, 1 / float(self.window.sample_rate) )
        if shifted:
            return np.fft.fftshift(freqs)
        else:
            return freqs

    def update(self):
        _fft = fft(self.window.window(),axis=0)
        freqs = self.freqs()
        # print np.real(_fft[:,0])
        _fft[abs(freqs) < self.cutoff,:] = 0
        # print abs(freqs) < self.cutoff
        # _fft[np.logical_and(freqs >= 0, freqs < self.cutoff),:] = 0
        # _fft[:,:] = 10
        # print np.logical_and((freqs >= 0), (freqs < self.cutoff))
        # print np.real(_fft[:,0])


        self.sparks = ""
        for i in xrange(_fft.shape[1]):
            self.sparks += str(i) + ", " + spark.getSparks( abs(np.hstack([np.real(np.fft.fftshift(_fft)[:,i]),[50]]) ) ) + "\n"

        self.filtered = np.real(ifft(_fft,axis=0))

class FrequencySpectrum(object):
    """docstring for FrequencySpectrum"""
    def __init__(self, window):
        super(FrequencySpectrum, self).__init__()
        self.window = window
    
    def spark_it(self):
        '''Generates frequency spectrum of channels in window with fft and prints it with spark'''
        _fft = np.fft.fftshift(fft(self.window.window(),axis=0))
        freqs = np.fft.fftshift(np.fft.fftfreq(self.window.window_size, 1 / float(self.window.sample_rate)))

        s = ""
        for i in xrange(_fft.shape[1]):
            s += str(i) + ", " + spark.getSparks( abs(np.hstack([np.real(_fft[:,i]),[50]]) ) ) + "\n"

        return s

        

class Node(object):
    """Subscribes to imu data with orientation and removes gravity by applying a digital high-pass-filter to the acceleration."""
    # http://docs.scipy.org/doc/numpy/reference/routines.fft.html

    def __init__(self, ):
        super(Node, self).__init__()
        rospy.init_node(basename(__file__).replace('.','_'))
        signal.signal(signal.SIGINT, self.keyboard_interupt)

        if rospy.get_param('~use_joystick',False):
            rospy.Subscriber('/joy/', Joy, self.callback_joystick)


        self.window_size = rospy.get_param('~window_size',10)
        self.console_output_config = rospy.get_param('~console_output',self._default_console_output_config())

        self.window = Window(self.window_size, 3)
        self.filter = HighPassSharpCutoff(self.window,20)

        self.filter_window = Window(self.window_size, 3)

        self.freqspec = FrequencySpectrum(self.window)
        self.freqspec_filter = FrequencySpectrum(self.filter_window)

        self.last_time = rospy.Time.now().to_sec()

        self.stop = False


        rospy.Subscriber('/accelerometer/raw', Vector3, self.callback)
        self.pub = rospy.Publisher('/accelerometer/high_pass', Vector3)

        rospy.spin()

    def _default_console_output_config(self):
        return dict({'enable': False,
                     'freqspec': True,
                     'freqspec_filtered': True,
                     'freqspec_filtered_output': True,
                     'frequencies': True,
                     'cutoff': True
                     })

    def generate_console_output(self):

        s = ["------------"]

        if self.console_output_config['freqspec']:
            s.append( self.freqspec.spark_it() )
            s.append( "------ " )

        if self.console_output_config['freqspec_filtered']:
            s.append( self.filter.sparks )

        if self.console_output_config['freqspec_filtered_output'] and self.filter_window.is_full():
            s.append( "------ " )
            s.append( self.freqspec_filter.spark_it() )

        if self.console_output_config['frequencies']:
            s.append( str(self.filter.freqs(shifted=True)) )

        if self.console_output_config['cutoff']:
            s.append( str(self.filter.cutoff) )

        return '\n'.join(s)

    def console_output(self):
        if self.console_output_config['enable']:
            print self.generate_console_output()

    def callback_joystick(self, joystick):
        dtime = rospy.Time.now().to_sec() - self.last_time

        # self.filter.cutoff = abs(joystick.axes[1]) * self.filter.maximum_freq()
        self.filter.cutoff = max(0.0,min(self.filter.maximum_freq(),self.filter.cutoff + (joystick.axes[1]) * dtime * 0.005))
        self.window_size = max(1.0,self.window_size + (-joystick.axes[3]) * dtime * 0.005)
        # print 'cutoff:', self.filter.cutoff, 'window_size:', self.window_size, 'maximum_freq:', self.filter.maximum_freq()
        # print self.window_size
        self.window.set_window_size(int(self.window_size))
        self.filter_window.set_window_size(int(self.window_size))

        self.last_joystick = joystick



    def callback(self, data):
        # print "sdlfz setou e 0z"
        if self.stop:
            return

        self.window.add_sample([data.x,data.y,data.z])

        # print self.window.pointer

        
        if self.window.is_full():
            self.console_output() # todo move 2 lines up

            self.filter.update()
            filtered = self.filter.get()
            self.filter_window.add_sample(filtered)
            self.pub.publish(Vector3(filtered[0],filtered[1],filtered[2]))

        
    def keyboard_interupt(self, signum, frame):
        rospy.signal_shutdown('keyboard interupt')
        sys.exit()


if __name__ == '__main__':
    Node()