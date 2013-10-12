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
        self.velocity = Vector3Stamped() # in pose frame
        self.pose = PoseStamped()
        # self.pose.pose.position.x = 0 #shouldn't be necessary
        # self.pose.pose.position.y = 0
        # self.pose.pose.position.z = 0

        rospy.Subscriber('/velocity', Vector3Stamped, self.callback_veloc)
        rospy.Subscriber('/imu/data', Imu, self.callback_imu)
        self.pub_pose = rospy.Publisher('/pose', PoseStamped)

        rospy.spin()


    def callback_imu(self, data):
        self.imu_data = data

    def callback_veloc(self, data):
        if self.first:
            self.velocity = data
            self.imu_data = data
            self.first = False
            return

        dtime = (data.header.stamp - self.velocity.header.stamp).to_sec() 
        self.velocity = data

        # transform self.velocity into global frame
        global_velocity = QuaternionAlg(self.imu_data.orientation).rotate_vector3(self.velocity.vector)

        self.pose.header = self.imu_data.header
        self.pose.pose.orientation = self.imu_data.orientation
        self.pose.pose.position.x += dtime * global_velocity.x
        self.pose.pose.position.y += dtime * global_velocity.y
        self.pose.pose.position.z += dtime * global_velocity.z

        self.pub_pose.publish(self.pose)

        self.run = False
        sys.exit()


    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."
        sys.exit()



if __name__ == '__main__':
    Node()