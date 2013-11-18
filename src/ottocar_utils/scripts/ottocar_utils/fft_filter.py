#!/usr/bin/env python
import rospy
from rospy import AnyMsg
# import ottocar_utils.Window
from ottocar_utils.window import Window
from ottocar_utils.str_to_class import str_to_class
from ottocar_utils.measure_sample_rate import MeasureSampleRate
from std_msgs.msg import *
from geometry_msgs.msg import *
from functools import partial
from os.path import basename

import numpy as np

class Filter(object):
    """docstring for Filter"""
    def __init__(self):
        super(Filter, self).__init__()

    def update(self, sample):
        pass

    def get(self):
        return False

class WindowedFilter(Filter):
    """docstring for WindowedFilter"""
    def __init__(self, window):
        super(WindowedFilter, self).__init__()

        self.window = window
        self.filtered = window.window()[:,:]

    def update(self, sample):
        pass

    def get(self):
        return self.filtered[-1,:]

class FFTFilter(WindowedFilter):
    """docstring for FFTFilter"""
    def __init__(self, window):
        super(FFTFilter, self).__init__(window)
        self.filter = self._pass

    def _sharp_cutoff_at(self, _cutoff, _fft, _freqs):
        print _cutoff, abs(_freqs) 
        print (abs(_freqs) <= _cutoff)
        _fft[abs(_freqs) <= _cutoff,:] = 0

    def sharp_cutoff_at(self, cutoff):
        print cutoff
        return lambda self,_fft, _freqs: self._sharp_cutoff_at(_cutoff=cutoff,_fft=_fft, _freqs=_freqs)
        # partial(self._sharp_cutoff_at, _cutoff=cutoff)
        # return partial(self._sharp_cutoff_at, _cutoff=cutoff)

    def _pass(self, _fft, _freqs):
        pass

    def maximum_freq(self):
        return float(self.window.sample_rate) / 2

    def freqs(self, shifted = False):
        self._freqs = np.fft.fftfreq(self.window.window_size, 1 / float(self.window.sample_rate) )

        if shifted:
            return np.fft.fftshift(self._freqs)
        else:
            return self._freqs

    def update(self):
        self._fft = np.fft.fft(self.window.window(),axis=0)
        freqs = self.freqs()
        # print np.real(_fft[:,0])

        # print self.filter
        self.filter(self,self._fft, freqs)

        # _fft[abs(freqs) < self.cutoff,:] = 0

        # print abs(freqs) < self.cutoff
        # _fft[np.logical_and(freqs >= 0, freqs < self.cutoff),:] = 0
        # _fft[:,:] = 10
        # print np.logical_and((freqs >= 0), (freqs < self.cutoff))
        # print np.real(_fft[:,0])


        # self.sparks = ""
        # for i in xrange(_fft.shape[1]):
            # self.sparks += str(i) + ", " + spark.getSparks( abs(np.hstack([np.real(np.fft.fftshift(_fft)[:,i]),[50]]) ) ) + "\n"

        self.filtered = np.real(np.fft.ifft(self._fft,axis=0))


class Node(object):
    """docstring for Node"""
    def __init__(self, topic_in='/topic_in', topic_out='/topic_out', publish_callback=False,
                topic_type='Vector3', window_size=10, 
                filter_name = "sharp_cutoff_at", filter_arg_0 = 0,
                measure_sample_rate_update_interval=10, measure_sample_rate_gain=0.5):
        super(Node, self).__init__() 
        self.topic_in = topic_in
        self.topic_out = topic_out
        self.topic_type = topic_type
        self.window_size = window_size
        self.filter_name = filter_name
        self.filter_arg_0 = filter_arg_0
        self.publish_callback = publish_callback
        self.measure_sample_rate_update_interval = measure_sample_rate_update_interval
        self.measure_sample_rate_gain = measure_sample_rate_gain

        self.measure = MeasureSampleRate(
            update_interval=self.measure_sample_rate_update_interval, 
            gain=self.measure_sample_rate_gain)

        self.types = dict({
            Float32:dict({
                "n_signals": 1,
                "select_signals": lambda msg: [msg.data],
                "publish": lambda signals: signals[0]
            }),
            Vector3:dict({
                "n_signals": 3,
                "select_signals": lambda msg: [msg.x,msg.y,msg.z],
                "publish": lambda signals: Vector3(x=signals[0],y=signals[1],z=signals[2])
            }),
            Vector3Stamped:dict({
                "n_signals": 3,
                "select_signals": lambda msg: [msg.vector.x,msg.vector.y,msg.vector.z],
                "publish": lambda signals: Vector3Stamped(vector=Vector3(x=signals[0],y=signals[1],z=signals[2]))
            })
        })


        # if topic_type != False:
        # print globals()

        if self.topic_type in globals():
            self.topic_type_class = globals()[self.topic_type]
        # print self.topic_type_class
        # self.topic_type_class = str_to_class(self.topic_type)
        if self.topic_type_class in self.types:
            # _n_signals = 0
            info = self.types[self.topic_type_class]
            # for (_type, info) in self.types.iteritems():
                # if isinstance(self.topic_type_class, (_type)):
            _n_signals = info["n_signals"]


            self.window = Window(window_size=self.window_size, n_signals=_n_signals)
            self.fft_filter = FFTFilter(self.window)
            if self.filter_name == "sharp_cutoff_at":
                # print "spfjodfsg", self.filter_arg_0
                self.fft_filter.filter = self.fft_filter.sharp_cutoff_at(self.filter_arg_0)
                # print self.fft_filter.filter
                # help(self.fft_filter.filter)
                # self.fft_filter.filter()

            if isinstance(self.topic_out, (str)):
                self.pub = rospy.Publisher(self.topic_out, self.topic_type_class)

                if self.publish_callback == False:
                    self.publish = self._publish
                else:
                    self.publish = self.publish_callback


            rospy.Subscriber(self.topic_in, self.topic_type_class, self.callback)
            rospy.spin()
    
    def _publish(self, msg):
        self.pub.publish(msg)

    def callback(self, msg):
        self.measure.add_sample()

        # for (_type, info) in self.types.iteritems():
        if self.topic_type_class in self.types:
            info = self.types[self.topic_type_class]
            # if isinstance(self.topic_type_class, (_type)):
            self.window.add_sample(info["select_signals"](msg));
    
            if self.window.is_full():
                self.fft_filter.update()
                filtered = self.fft_filter.get()

                self.publish(info["publish"](filtered))
                # self.pub.publish(Vector3(filtered[0],filtered[1],filtered[2]))
                
                # break

        # self.measure.sample_rate
        # self.pub.publish(Float32(data=self.measure.sample_rate))
        
            
__all__ = ['MeasureSampleRate']

if __name__ == '__main__':
    rospy.init_node(basename(__file__).replace('.','_'))
    Node(topic_type=rospy.get_param('~topic_type', 'Vector3'),
        window_size = rospy.get_param('~window_size',10),
        filter_name = rospy.get_param('~filter', 'sharp_cutoff_at'), filter_arg_0 = rospy.get_param('~filter_arg_0', 0),
        measure_sample_rate_update_interval=rospy.get_param('~measure_sample_rate_update_interval', 10),
        measure_sample_rate_gain=rospy.get_param('~measure_sample_rate_gain', 0.5))