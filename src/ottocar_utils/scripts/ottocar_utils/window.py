#!/usr/bin/env python
import rospy
from ottocar_utils.measure_sample_rate import MeasureSampleRate
import numpy as np

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
