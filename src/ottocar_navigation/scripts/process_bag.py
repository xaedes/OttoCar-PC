#!/usr/bin/env python
# coding=utf8

import numpy as np
import matplotlib.pyplot as plt

import rosbag

from std_msgs.msg import Float32 
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped


class Buffer(object):
    """Stores values of a specific ros msg type in a dynamic numpy array"""
    def __init__(self, msg_type, initial_size = 100, growth_factor = 1.5, auto_timedeltas = True):
        super(Buffer, self).__init__()
        self.msg_type = msg_type
        self.initial_size = initial_size
        self.growth_factor = growth_factor
        self.auto_timedeltas = auto_timedeltas

        self.msg_types = dict({
            Float32:dict({
                "n": 2,
                "to_arr": lambda msg, timestamp: [timestamp.to_sec(), msg.data],
                "to_msg": lambda signals: Float32(signals[0])
            }),
            Vector3:dict({
                "n": 4,
                "to_arr": lambda msg, timestamp: [timestamp.to_sec(), msg.x,msg.y,msg.z],
                "to_msg": lambda signals: Vector3(x=signals[0],y=signals[1],z=signals[2])
            }),
            Vector3Stamped:dict({
                "n": 4,
                "to_arr": lambda msg, timestamp: [msg.header.stamp.to_sec(), msg.vector.x,msg.vector.y,msg.vector.z],
                "to_msg": lambda signals: Vector3Stamped(vector=Vector3(x=signals[0],y=signals[1],z=signals[2]))
            })
        })

        self.n = self.msg_types[self.msg_type]["n"]
        self._arr = np.zeros(shape=(self.initial_size,self.n))
        self._to_arr = self.msg_types[self.msg_type]["to_arr"]
        self._to_msg = self.msg_types[self.msg_type]["to_msg"]
        self.size = 0

        self.data = None
        self.values = None
        self.timestamps = None
        if self.auto_timedeltas:
            self.timedeltas = None
        self._update_vars()

    @staticmethod
    def like(buf):
        return Buffer(msg_type=buf.msg_type, 
            initial_size=buf.size, 
            growth_factor=buf.growth_factor, 
            auto_timedeltas=buf.auto_timedeltas)

    @staticmethod
    def clone(buf):
        #disable warning about access to protected members
        #pylint: disable=W0212  
        _clone = Buffer(msg_type=buf.msg_type, 
            initial_size=buf._arr.shape[0], 
            growth_factor=buf.growth_factor, 
            auto_timedeltas=buf.auto_timedeltas)
        _clone._arr[:,:] = buf._arr
        _clone.size = buf.size
        _clone._update_vars()
        return _clone

    def add_zero(self, timestamp):
        self.add_arr(np.hstack([[timestamp],np.zeros(shape=(self.n-1))]))

    def add_msg(self, msg, timestamp):
        self.add_arr(self._to_arr(msg,timestamp))

    def add_arr(self, arr):
        self._dynamic_size()
        self._arr[self.size,:] = arr
        self.size += 1
        self._update_vars()

    def _dynamic_size(self):
        while self.size >= self._arr.shape[0]:
            self._arr = np.vstack([self._arr, np.zeros(shape=(int(self._arr.shape[0] * self.growth_factor)  - self._arr.shape[0],self.n))])

    def _update_vars(self):
        self.data = self._arr[:self.size,:]
        self.timestamps = self._arr[:self.size,0]
        if self.auto_timedeltas:
            if self.timestamps.shape[0] == 0:
                self.timestamps = np.zeros(shape=(0,self.n))
            else:
                self.timedeltas = self.timestamps - np.roll(self.timestamps,1,axis=0)
                self.timedeltas[0] = 0
        self.values = self._arr[:self.size,1:]



a_raw = Buffer(Vector3)

# read rosbag file
bag_fn = 'bags/testlauf 01 - 15.11.2013/run-01-2013-11-15-18-08-36.bag' # gradlinige beschleunigung mit anschließendem abbremsen
# bag_fn = 'bags/testlauf 01 - 15.11.2013/run-02-2013-11-15-18-12-02.bag' # kreisbewegung (ccw)
# bag_fn = 'bags/testlauf 01 - 15.11.2013/run-03-2013-11-15-19-51-47.bag' # kreisbewegung mit anfahrt (ccw)
bag = rosbag.Bag(bag_fn)
for topic, _msg, ts in bag.read_messages(topics=['/accelerometer/raw']):
    a_raw.add_msg(_msg, ts)
bag.close()

def low_pass(buf, alpha = 0.005):
    # apply low_pass (from http://en.wikipedia.org/wiki/Low-pass_filter)
    result = Buffer.like(buf)
    result.add_arr(buf.data[0,:])
    for i in xrange(1,buf.size):
        arr = result.values[-1,:] + alpha * (buf.values[i,:]-result.values[-1,:])
        arr = np.hstack([[buf.timestamps[i]],arr])
        result.add_arr(arr)
    return result

def high_pass(buf, alpha = 0.005, start_with_zero = False):
    # apply high_pass (from http://en.wikipedia.org/wiki/High-pass_filter#Algorithmic_implementation)
    result = Buffer.like(buf)
    if start_with_zero:
        result.add_zero(buf.timestamps[0])
    else:
        result.add_arr(buf.data[0,:])
    for i in xrange(1,buf.size):
        arr = alpha * (result.values[-1,:] + buf.values[i,:] - buf.values[-1,:])
        arr = np.hstack([[buf.timestamps[i]],arr])
        result.add_arr(arr)
    return result

def smooth(buf, alpha = 0.5):
    # apply high_pass (from http://en.wikipedia.org/wiki/High-pass_filter#Algorithmic_implementation)
    result = Buffer.like(buf)
    result.add_arr(buf.data[0,:])
    for i in xrange(1,buf.size):
        arr = alpha * buf.values[i,:] + (1-alpha) * result.values[-1,:]
        arr = np.hstack([[buf.timestamps[i]],arr])
        result.add_arr(arr)
    return result

def integrate(buf):
    result = Buffer.like(buf)
    result.add_zero(buf.timestamps[0])
    for i in xrange(1,buf.size):
        arr = result.values[-1,:] + buf.values[i,:] * buf.timedeltas[i]
        arr = np.hstack([[buf.timestamps[i]],arr])
        result.add_arr(arr)
    return result        

a_low_pass = low_pass(a_raw, alpha = 0.0005)
a_wo_grav_lp = Buffer.clone(a_raw)
a_wo_grav_lp.values -= a_low_pass.values
a_wo_grav_hp = high_pass(a_raw, alpha = 0.5, start_with_zero = True)

a_wo_grav_lp_smooth = smooth(a_wo_grav_lp, alpha = 0.15)
a_wo_grav_hp_smooth = smooth(a_wo_grav_hp, alpha = 0.15)

# integrate to velocity
v_lp = integrate(a_wo_grav_lp_smooth)
v_hp = integrate(a_wo_grav_hp_smooth)

# integrate to position
p_lp = integrate(v_lp)
p_hp = integrate(v_hp)

plt.figure(1)

plt.subplot(4,2,1)
plt.plot(a_wo_grav_lp.timestamps, a_wo_grav_lp.values)

plt.subplot(4,2,2)
plt.plot(a_wo_grav_hp.timestamps, a_wo_grav_hp.values)

plt.subplot(4,2,3)
plt.plot(a_wo_grav_lp_smooth.timestamps, a_wo_grav_lp_smooth.values)

plt.subplot(4,2,4)
plt.plot(a_wo_grav_hp_smooth.timestamps, a_wo_grav_hp_smooth.values)

plt.subplot(4,2,5)
plt.plot(v_lp.timestamps, v_lp.values)

plt.subplot(4,2,6)
plt.plot(v_hp.timestamps, v_hp.values)

plt.subplot(4,2,7)
plt.plot(p_lp.timestamps, p_lp.values)

plt.subplot(4,2,8)
plt.plot(p_hp.timestamps, p_hp.values)



plt.show()


