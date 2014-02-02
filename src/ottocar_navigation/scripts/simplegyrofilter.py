#!/usr/bin/env python


from time import sleep
import signal

import rospy
import os
import sys
import math

from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu


class Node(object):



    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."
        sys.exit()

    # def spin(self):
    #     r = rospy.Rate(40) # in [hz]
    #     while(self.run):

    #         r.sleep()


    def callback_speed_cmd(self,data):
        self.speed_cmd = data.data
        self.last_speed_time = self.time()

    def callback_rsp_cmd(self,data):
        self.rsp_cmd = data.data
        self.last_speed_time = self.time()


    def time(self):
        return rospy.Time.now().to_sec()

    def callback_imu(self,data):
        self.imu = data

        if(self.time()-self.last_speed_time >= self.speed_timeout):
            self.speed_cmd = 0
            self.last_speed_time = self.time()

        if(self.rsp_cmd==0):
            self.gyrobias = self.biasgain * self.imu.angular_velocity.z + (1-self.biasgain) * self.gyrobias

        self.gyroz = self.gain * (self.imu.angular_velocity.z-self.gyrobias) + (1-self.gain) * self.gyroz


        v = Vector3Stamped()
        v.vector.z = self.gyroz

        self.pub_gyro.publish(v)
        


    def __init__(self):
        super(Node, self).__init__()
        self.run = True
        self.pause = False
        signal.signal(signal.SIGINT, self.keyboard_interupt)  

        rospy.init_node('simplegyrofilter')


#        rospy.Subscriber('/speed_cmd', Int8, self.callback_speed_cmd)
        rospy.Subscriber('/rsp_cmd', Int8, self.callback_speed_cmd)
        rospy.Subscriber('/imu/data_raw', Imu, self.callback_imu)

        self.speed_timeout = 2

        self.gyroz = 0
        self.gain = 0.995
        # self.gain = 0.5
        self.biasgain = 0.995
        self.gyrobias = 0
        self.last_speed_time = 0
	self.speed_cmd=1
	self.rsp_cmd=1

        # publisher for messages
        self.pub_gyro = rospy.Publisher('/gyro/filtered', Vector3Stamped, tcp_nodelay=True)
        
        rospy.spin()

if __name__ == '__main__':
    Node()
