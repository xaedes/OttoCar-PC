#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
import sys

import numpy as np


# def callback(data):
#     rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)


# def talker():
#     pub = rospy.Publisher('chatter', String)
#     rospy.init_node('talker')
#     while not rospy.is_shutdown():
#         str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(str)
#         pub.publish(String(str))
#         rospy.sleep(1.0)

# def listener():

class Node(object):
    """docstring for Foo"""

    def callback(self, data):
        self.buffer[self.pointer,0] = data.x
        self.buffer[self.pointer,1] = data.y
        self.buffer[self.pointer,2] = data.z
        self.pointer = (self.pointer + 1) % self.window_size

        std = self.buffer.std(axis=0)
        vec = Vector3(std[0],std[1],std[2])
        self.pub.publish(vec)

    def __init__(self):
        super(Node, self).__init__()

        rospy.init_node('moving_standard_deviation', anonymous=True)

        self.window_size = rospy.get_param('~window_size',100)

        self.buffer = np.zeros(shape=(self.window_size,3))
        self.pointer = 0

        self.pub = rospy.Publisher(rospy.get_param('~topic_out','/accelerometer/standard_deviation'), Vector3)

        # subscribe to imu data
        rospy.Subscriber(rospy.get_param('~topic_in','/accelerometer/raw'), Vector3, self.callback)

        rospy.spin()




if __name__ == '__main__':
    try:
        Node()



    except rospy.ROSInterruptException:
        pass


