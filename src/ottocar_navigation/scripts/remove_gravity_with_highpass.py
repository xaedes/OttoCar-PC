#!/usr/bin/env python

import pygame
import signal

import math
import sys

from os.path import basename

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


class Node(object):
    """Subscribes to imu data with orientation and removes gravity by applying a high pass filter to the acceleration."""

    def __init__(self, ):
        super(Node, self).__init__()
        rospy.init_node(basename(__file__).replace('.','_'))

        signal.signal(signal.SIGINT, self.keyboard_interupt)  

        self.imu_data = Imu()
        self.gravity = Vector3(0,0,9.81)

        self.high_pass = Vector3()

        rospy.Subscriber('/imu/data', Imu, self.callback_imu)
        rospy.Subscriber('/accelerometer/high_pass', Vector3, self.callback_highpass)
        self.pub_imu = rospy.Publisher('/imu/data_wo_gravity', Imu)


        rospy.spin()

    def callback_highpass(self, data):
        self.high_pass = data

    def callback_imu(self, data):
        self.imu_data = data

        self.imu_data.linear_acceleration = self.high_pass

        self.pub_imu.publish(self.imu_data)
        
    def keyboard_interupt(self, signum, frame):
        sys.exit()


if __name__ == '__main__':
    Node()