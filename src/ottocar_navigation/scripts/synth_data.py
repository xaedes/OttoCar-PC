#!/usr/bin/env python

import pygame
from time import sleep
import signal


from os.path import basename

from math import pi,sin,cos

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
        rospy.init_node(basename(__file__).replace('.','_'))
        signal.signal(signal.SIGINT, self.keyboard_interupt)  
        self.run = True

        self.publish_rate = rospy.get_param('~publish_rate', 5)   # in hz
        self.interval =  1 / self.publish_rate
     
        self.pub_imu = rospy.Publisher('/synth/imu', Imu)
        self.pub_pose = rospy.Publisher('/synth/pose', PoseStamped)

        self.start_time = self.last_time = rospy.Time.now().to_sec()
        self.spin()

    def circular_path(self, radius = 1, angular_speed = 2*pi / 10, center = Vector3(0,0,0) ):
        # move along a circular path in xy-plane

        # init msgs
        stamped = PoseStamped()
        stamped.header.stamp = rospy.Time.now()
        stamped.header.frame_id = "/synth"

        # position on circle
        angle = angular_speed * self.total_time
        stamped.pose.position.x = center.x + cos(angle) * radius
        stamped.pose.position.y = center.y + sin(angle) * radius
        stamped.pose.position.z = center.z

        # orientation [todo]
        # x-axis tangential to movement 
        # y-axis pointing to the center
        # z-axis pointing upwards 
        quat = QuaternionAlg.identity() 
        quat *= QuaternionAlg.from_axis(rad=angle+pi/2,axis=Vector3(x=0,y=0,z=1))
        stamped.pose.orientation = quat.as_rosquaternion()



        self.pub_pose.publish(stamped)

    def update(self):
        now = rospy.Time.now().to_sec()
        dtime = now - self.last_time  # delta time in seconds
        self.last_time = now

        self.total_time = now - self.start_time

        self.circular_path()
               



        # # update position

        # # apply orientation to velocity 
        # #  http://content.gpwiki.org/index.php/OpenGL%3aTutorials%3aUsing_Quaternions_to_represent_rotation#Rotating_vectors
        # #  rotate velocity by pose quaternion and add it to position

        # velocity_quat = QuaternionAlg(w=0,x=self.velocity.x,y=self.velocity.y,z=self.velocity.z)
        # quat = Node.fromROSQuat(self.pose.orientation)
        # rotated = quat * velocity_quat * ~quat

        # self.pose.position.x += rotated.x * dtime
        # self.pose.position.y += rotated.y * dtime
        # self.pose.position.z += rotated.z * dtime

        # print self.pose
        # stamped = PoseStamped()
        # stamped.pose = self.pose
        # stamped.header.stamp = rospy.Time.now()
        # stamped.header.frame_id = "/odom"
        # self.pub_pose.publish(stamped)

        
    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."

    def spin(self):
        self.clock = pygame.time.Clock()
        self.clock.tick()

        while(self.run):
            start = rospy.Time.now().to_sec()
            self.update()
            update_time = rospy.Time.now().to_sec() - start   # time needed for update
            sleep_time = self.interval - update_time               
            if sleep_time > 0:
                sleep(sleep_time)                      

if __name__ == '__main__':
    Node()