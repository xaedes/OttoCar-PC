#!/usr/bin/env python

import pygame
from time import sleep
import signal

import sys
import math

from os.path import basename

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

from quaternion import Quaternion as QuaternionAlg

class Node(object):
    """docstring for Node"""

    def __init__(self, ):
        super(Node, self).__init__()
        rospy.init_node(basename(__file__).replace('.','_'))

        signal.signal(signal.SIGINT, self.keyboard_interupt)  
        self.run = True
        self.first = True


        self.imu_data = Imu()

        # velocity in pose frame
        self.velocity = Vector3Stamped()
        self.velocity.vector.x = 0
        self.velocity.vector.y = 0
        self.velocity.vector.z = 0



        rospy.Subscriber('/imu/data_wo_gravity', Imu, self.callback_imu)
        self.pub_veloc = rospy.Publisher('/velocity', Vector3Stamped)

        rospy.spin()


    def callback_imu(self, data):
        if self.first:
            self.imu_data = data
            self.first = False
            return

        dtime = (data.header.stamp - self.imu_data.header.stamp).to_sec() 
        self.imu_data = data

        self.velocity.header = self.imu_data.header
        self.velocity.vector.x += dtime * self.imu_data.linear_acceleration.x
        self.velocity.vector.y += dtime * self.imu_data.linear_acceleration.y
        self.velocity.vector.z += dtime * self.imu_data.linear_acceleration.z

        # print dtime
        # print dtime * self.imu_data.linear_acceleration.x
        # print dtime * self.imu_data.linear_acceleration.y
        # print dtime * self.imu_data.linear_acceleration.z
        # print self.velocity.vector.x
        # print self.velocity.vector.y
        # print self.velocity.vector.z

        self.pub_veloc.publish(self.velocity)

        self.run = False
        sys.exit()


    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."
        sys.exit()



if __name__ == '__main__':
    Node()