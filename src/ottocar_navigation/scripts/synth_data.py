#!/usr/bin/env python

import pygame
from time import sleep
import signal


from os.path import basename

from math import pi,sin,cos,sqrt

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

        self.publish_rate = rospy.get_param('~publish_rate', 100)   # in hz
        self.interval =  1.0 / self.publish_rate
     
        self.pub_imu = rospy.Publisher('/synth/imu', Imu)
        self.pub_veloc = rospy.Publisher('/synth/velocity', Vector3)
        self.pub_pose = rospy.Publisher('/synth/pose', PoseStamped)
        self.pub_mag = rospy.Publisher('/synth/mag', Vector3Stamped)

        self.start_time = self.last_time = rospy.Time.now().to_sec()
        self.spin()

    def ease_sin_inout(self, t, start=0, change=1, duration=1):
        return -(change/2)*(cos(pi*t/duration)-1)+start

    def integrated_ease_sin_inout(self, t, start=0, change=1, duration=1):
        return (change/2)*(t-(duration/pi)*(sin(pi*t/duration)+1))+start*t



    def circular_path(self, radius = 1, angular_speed_increase_time = 15, max_angular_speed = 1*2*pi / 5, center = Vector3(0,0,0) ):
        # move along a circular path in xy-plane

        # init msgs
        stamped = PoseStamped()
        stamped.header.stamp = rospy.Time.now()
        stamped.header.frame_id = "/synth"
        mag = Vector3Stamped()
        mag.header = stamped.header
        imu = Imu()
        imu.header = stamped.header


        if self.total_time < angular_speed_increase_time:
            # use sinusoidal in/out ease to increase angular_speed
            angular_speed = self.ease_sin_inout(self.total_time, change=max_angular_speed, duration=angular_speed_increase_time)
        else:
            angular_speed = max_angular_speed

        # position on circle
        if self.total_time < angular_speed_increase_time:
            # integration of speed easing
            angle = self.integrated_ease_sin_inout(self.total_time, change=max_angular_speed, duration=angular_speed_increase_time)
        else:
            angle = self.integrated_ease_sin_inout(angular_speed_increase_time, change=max_angular_speed, duration=angular_speed_increase_time)
            angle += angular_speed * (self.total_time - angular_speed_increase_time)

        stamped.pose.position.x = center.x + cos(angle) * radius
        stamped.pose.position.y = center.y + sin(angle) * radius
        stamped.pose.position.z = center.z

        # orientation [todo]
        # x-axis tangential to movement 
        # y-axis pointing to the center
        # z-axis pointing upwards 
        quat = QuaternionAlg.identity() 
        quat *= QuaternionAlg.from_axis(
            rad=angle+pi/2,
            axis=Vector3(x=0,y=0,z=1))
        stamped.pose.orientation = quat.as_rosquaternion()
        
        velocity = Vector3()
        perimeter = 2*pi*radius
        rpm = angular_speed / (2*pi)
        speed = perimeter * rpm

        velocity = quat.inv_rotate_vector3( Vector3(0,speed,0) )

        # project north to global y-xis
        mag.vector =  quat.inv_rotate_vector3(Vector3(x=0,y=1,z=0))

        imu.orientation = stamped.pose.orientation
        imu.angular_velocity.x = 0
        imu.angular_velocity.y = 0
        imu.angular_velocity.z = angular_speed

        vec = Vector3()
        vec.z = -9.81
        if self.total_time < angular_speed_increase_time:
            # after doing the math (second derivative of position calculation) i came up with following:
            vec.x = -radius * (cos(angle)*angular_speed*angular_speed + (max_angular_speed*pi/
                (2*angular_speed_increase_time)) * sin(angle) * sin(pi*self.total_time/angular_speed_increase_time) )
            vec.y = radius * (-sin(angle)*angular_speed*angular_speed + (max_angular_speed*pi/
                (2*angular_speed_increase_time)) * cos(angle) * sin(pi*self.total_time/angular_speed_increase_time) )

                
        else:
            # acceleration on uniform circular motion points towards the mid
            # http://de.wikipedia.org/wiki/Gleichf%C3%B6rmige_Kreisbewegung
            abs_accel = radius * angular_speed * angular_speed
            
            vec.x = - cos(angle) * abs_accel
            vec.y = - sin(angle) * abs_accel

        imu.linear_acceleration = quat.inv_rotate_vector3( vec )


        self.pub_imu.publish(imu)
        self.pub_veloc.publish(velocity)
        self.pub_pose.publish(stamped)
        self.pub_mag.publish(mag)

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