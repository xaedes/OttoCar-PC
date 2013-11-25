#!/usr/bin/env python
import rospy
from os.path import basename
from hashlib import sha1
from random import random


import numpy as np

from std_msgs.msg import Float32 
from geometry_msgs.msg import Vector3, Vector3Stamped



class Node(object):
    def __init__(self, topic_in='/topic_in', topic_out='/topic_out', publish_callback=None,
                topic_type='Vector3'):
        super(Node, self).__init__() 
        self.topic_in = topic_in
        self.topic_out = topic_out
        self.topic_type = topic_type
        self.publish_callback = publish_callback

        # Define supported types
        self.types = dict({
            Float32:dict({
                "n_signals": 1,
                "select_signals": lambda msg: [msg.data],
                "publish": lambda signals: signals[0],
                "time": lambda msg: rospy.Time.now().to_sec()
            }),
            Vector3:dict({
                "n_signals": 3,
                "select_signals": lambda msg: [msg.x,msg.y,msg.z],
                "publish": lambda signals: Vector3(x=signals[0],y=signals[1],z=signals[2]),
                "time": lambda msg: rospy.Time.now().to_sec()
            }),
            Vector3Stamped:dict({
                "n_signals": 3,
                "select_signals": lambda msg: [msg.vector.x,msg.vector.y,msg.vector.z],
                "publish": lambda signals: Vector3Stamped(vector=Vector3(x=signals[0],y=signals[1],z=signals[2])),
                "time": lambda msg: msg.header.stamp.to_sec()
            })
        })

        # for calculation of delta time
        self.last_time = None


        # get class from topic_type
        if self.topic_type in globals():
            self.topic_type_class = globals()[self.topic_type]

        # if topic_type is implemented
        if self.topic_type_class in self.types:
            # get info for topic_type
            info = self.types[self.topic_type_class]
            self._n_signals = info["n_signals"]

            # initialize buffer for data to be stored
            self.integrated = np.zeros(shape=(self._n_signals))

            # if results shall be published, set up publisher
            if isinstance(self.topic_out, (str)):
                self.pub = rospy.Publisher(self.topic_out, self.topic_type_class)

            # if publish callback is given use it, else use publisher
            if self.publish_callback is not None:
                self.publish = self.publish_callback
            else:
                self.publish = self._publish

            # lets get the party started
            rospy.Subscriber(self.topic_in, self.topic_type_class, self.callback)
            rospy.spin()
    
    def _publish(self, msg):
        self.pub.publish(msg)

    def callback(self, msg):
        if self.topic_type_class in self.types:
            info = self.types[self.topic_type_class]
            time = info["time"](msg)
            if self.last_time is None:
                self.last_time = time

            dtime = time - self.last_time

            self.integrated += np.array(info["select_signals"](msg)) * dtime

            self.last_time = time

            self.publish(info["publish"](self.integrated))
        
            
if __name__ == '__main__':
    rospy.init_node(basename(__file__).replace('.','_')+sha1(str(random())).hexdigest()[-6:])
    Node(topic_type=rospy.get_param('~topic_type', 'Vector3'),
        )