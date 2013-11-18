#!/usr/bin/env python
import rospy
from rospy import AnyMsg
from std_msgs.msg import Float32
from os.path import basename

class MeasureSampleRate(object):
    """docstring for MeasureSampleRate"""
    def __init__(self, update_interval = 10, gain = 0.5):
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

        return self.sample_rate

    def __complex__(self):
        return complex(self.sample_rate)
    def __int__(self):
        return int(self.sample_rate)
    def __long__(self):
        return long(self.sample_rate)
    def __float__(self):
        return float(self.sample_rate)

class Node(object):
    """docstring for Node"""
    def __init__(self, topic_in='/topic_in', topic_out='/topic_out', publish_rate=False, update_interval=10, gain=0.5):
        super(Node, self).__init__()
        self.topic_in = topic_in
        self.topic_out = topic_out
        self.publish_rate = publish_rate
        self.update_interval = update_interval
        self.gain = gain
        rospy.init_node(basename(__file__).replace('.','_'))
        self.measure = MeasureSampleRate(update_interval= self.update_interval, gain=self.gain)

        # topic_type = rospy.get_param('~topic_type', False);
        # if topic_type != False:
            # topic_type_class = str_to_class(topic_type)
        

        self.pub = rospy.Publisher(self.topic_out, Float32)
        
        self.publish_rate = publish_rate
        
        if self.publish_rate == False:
            rospy.Subscriber(self.topic_in, AnyMsg, self.callback_immediate_publish)
            rospy.spin()
            
        elif isinstance(self.publish_rate, (int, long, float)):
            rospy.Subscriber(self.topic_in, AnyMsg, self.callback_publish_later)
            self._r = rospy.Rate(self.publish_rate) 
            while not rospy.is_shutdown():
                self.pub.publish(Float32(data=self.measure.sample_rate))
                self._r.sleep()

    def callback_immediate_publish(self, msg):
        self.measure.add_sample()
        self.pub.publish(Float32(data=self.measure.sample_rate))
        

    def callback_publish_later(self, msg):
        self.measure.add_sample()
            
__all__ = ['MeasureSampleRate']

if __name__ == '__main__':
    Node(publish_rate=rospy.get_param('~publish_rate', False),
        update_interval=rospy.get_param('~update_interval', 10),
        gain=rospy.get_param('~gain', 0.5))