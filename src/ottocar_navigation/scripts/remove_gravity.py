#!/usr/bin/env python

import pygame
from time import sleep
import signal

import math
import sys

from os.path import basename

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

from quaternion import Quaternion as QuaternionAlg

class Node(object):
    """Subscribes to imu data with orientation and removes gravity by substracting it from acceleration."""

    def __init__(self, ):
        super(Node, self).__init__()
        rospy.init_node(basename(__file__).replace('.','_'))

        signal.signal(signal.SIGINT, self.keyboard_interupt)  

        self.imu_data = Imu()
        self.gravity = Vector3(0,0,9.81)

        rospy.Subscriber('/imu/data', Imu, self.callback_imu)
        self.pub_imu = rospy.Publisher('/imu/data_wo_gravity', Imu)


        rospy.spin()

    def callback_imu(self, data):
        self.imu_data = data

        # transform gravity from global frame in imu frame:
        #  rotate gravity (0,0,-g) by the inverse of the orientation of the imu 
        transformed_gravity = QuaternionAlg(self.imu_data.orientation).inv_rotate_vector3(self.gravity)
        self.imu_data.linear_acceleration.x -= transformed_gravity.x
        self.imu_data.linear_acceleration.y -= transformed_gravity.y
        self.imu_data.linear_acceleration.z -= transformed_gravity.z
            
        self.pub_imu.publish(self.imu_data)
        
    def keyboard_interupt(self, signum, frame):
        sys.exit()


if __name__ == '__main__':
    Node()