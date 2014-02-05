#!/usr/bin/env python
import rospy
# from time import time
from std_msgs.msg import Int32
from std_msgs.msg import UInt32
from std_msgs.msg import Float32

class Subscriber(object):
    """docstring for Subscriber"""
    def __init__(self):
        super(Subscriber, self).__init__()
        rospy.init_node('calc_rps', anonymous=True)

        self.gain = 0.25
        self.pub_rps = rospy.Publisher('/sensor_motor_rps', Float32)
        # self.pub_dt = rospy.Publisher('/dt', Float32)
        # self.pub_int = rospy.Publisher('/rps_integrate', Float32)

        self.time = None
        self.last_time = None
        self.dt = None

        self.revolutions_last = None
        self.rps = 0

        self.integrated = 0

        rospy.Subscriber('/sensor_motor_revolutions', Int32, self.callback_revolutions)
        rospy.Subscriber('/uC_time', UInt32, self.callback_uCTime)
        rospy.spin()

    def callback_uCTime(self, msg):
        self.last_time = self.time
        self.time = msg.data / 1000.
        if(self.last_time != None):
            self.dt = self.time - self.last_time

    def callback_revolutions(self, msg):
        revolutions = msg.data

        if(self.revolutions_last == None):
            self.revolutions_last = revolutions

        if((self.dt != None)and(self.dt>0)):
            current_rps = (revolutions - self.revolutions_last)/self.dt
            if(current_rps > 10000):    #must be wrong
                current_rps = self.rps
            self.rps = self.gain * current_rps + (1-self.gain) * self.rps

            self.integrated += self.dt * self.rps

            self.pub_rps.publish(self.rps)
            # self.pub_dt.publish(self.dt)
            # self.pub_int.publish(self.integrated)

        self.revolutions_last = revolutions

if __name__ == '__main__':
    Subscriber()