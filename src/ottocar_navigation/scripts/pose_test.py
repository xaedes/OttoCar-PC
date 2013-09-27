#!/usr/bin/env python

import pygame
from time import sleep
import signal

import math

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

from quaternion import Quaternion as QuaternionAlg

class Node(object):
    """docstring for Node"""

    def __init__(self, ):
        super(Node, self).__init__()
        rospy.init_node('pose_test')

        signal.signal(signal.SIGINT, self.keyboard_interupt)  
        self.run = True

        self.pose = Pose()
        self.pose.position.x = 0
        self.pose.position.y = 0
        self.pose.position.z = 0

        #identity quaternion
        self.pose.orientation.w = 1    
        self.pose.orientation.x = 0
        self.pose.orientation.y = 0
        self.pose.orientation.z = 0

        # #rotate around Z by 90 degree
        # self.pose.orientation.w = math.sqrt(0.5)
        # self.pose.orientation.x = 0
        # self.pose.orientation.y = 0
        # self.pose.orientation.z = math.sqrt(0.5)

        # velocity in pose frame
        self.velocity = Vector3(1,0,0)

        # print ~Node.fromROSQuat(self.pose.orientation)

        rospy.Subscriber('/imu/data', Imu, self.callback_imu)
        self.pub_pose = rospy.Publisher('pose', PoseStamped)


        self.spin()

    @staticmethod
    def fromROSQuat(rosQuat):
        return QuaternionAlg(w=rosQuat.w, x=rosQuat.x, y=rosQuat.y, z=rosQuat.z)

    @staticmethod
    def toROSQuat(quat):
        return Quaternion(w=quat.w, x=quat.x, y=quat.y, z=quat.z)

    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."


    def spin(self):
        self.clock = pygame.time.Clock()
        self.clock.tick()

        while(self.run):
            self.update()
            sleep(0.01)

        # rospy.spin()

    def update(self):
        dtime = self.clock.get_time() / 1000.0  # delta time in seconds
        self.clock.tick()
    	
    	# update position

    	# apply orientation to velocity 
    	#  http://content.gpwiki.org/index.php/OpenGL%3aTutorials%3aUsing_Quaternions_to_represent_rotation#Rotating_vectors
    	#  rotate velocity by pose quaternion and add it to position

    	velocity_quat = QuaternionAlg(w=0,x=self.velocity.x,y=self.velocity.y,z=self.velocity.z)
    	quat = Node.fromROSQuat(self.pose.orientation)
    	rotated = quat * velocity_quat * ~quat

    	self.pose.position.x += rotated.x * dtime
    	self.pose.position.y += rotated.y * dtime
    	self.pose.position.z += rotated.z * dtime

    	print self.pose
    	stamped = PoseStamped()
    	stamped.pose = self.pose
    	stamped.header.stamp = rospy.Time.now()
    	stamped.header.frame_id = "/odom"
    	self.pub_pose.publish(stamped)

    def callback_imu(self, data):
    	self.pose.orientation = data.orientation
        

if __name__ == '__main__':
    Node()