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
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

class Node(object):



    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."
        sys.exit()

    def control(self,last,gyro,soll):

        e = gyro - soll
        v = last + self.kps[self.mode] * e
        # print gyro
        print e,v,self.kps[self.mode]
        return v

    def control2(self,last,gyro,soll, dt):

        e = gyro - soll
        self.esums[self.mode] += e
        self.esums[self.mode] = min(max(self.esums[self.mode],-2),2)


        v = last + self.kps[self.mode] * e + self.kis[self.mode] * self.esums[self.mode] * dt
        print self.esums[self.mode]
        # print gyro
        # print e,v,self.kps[self.mode]
        return v


    def spin(self):
        r = rospy.Rate(20) # in [hz]
        lasttime = self.time()
        while(self.run):
            dt = self.time() - lasttime

            if(self.speed_cmd<0):
                self.mode = 0
                self.soll = -self.angle2_cmd
            else:
                self.mode = 1
                self.soll = self.angle2_cmd

            if(self.speed_cmd==0):
                self.esums = [0,0]

            self.angle_cmd = self.control2(self.angle_cmd,self.gyroz,self.soll,dt)
            # self.angle_cmd = self.control(self.angle_cmd,self.gyroz,self.soll)

            self.angle_cmd = min(max(self.angle_cmd,-126),126)

            # math.copysign(self.speed_cmd,self.kp)


            # print self.angle_cmd
            self.pub_angle.publish(self.angle_cmd)

            lasttime = self.time()
            r.sleep()

    def callback_speed_cmd(self,data):
        self.speed_cmd = data.data
    def callback_angle2_cmd(self,data):
        self.angle2_cmd = data.data


    def callback_gyro(self,data):
        self.gyroz = data.vector.z
        # print data
        
    def time(self):
        return rospy.Time.now().to_sec()

    def __init__(self):
        super(Node, self).__init__()
        self.run = True
        self.pause = False
        signal.signal(signal.SIGINT, self.keyboard_interupt)  

        rospy.init_node('geradeaus')


        # rospy.Subscriber('/imu/data_raw', Imu, self.callback_imu)
        rospy.Subscriber('/gyro/filtered', Vector3Stamped, self.callback_gyro)
        rospy.Subscriber('/speed_cmd', Int8, self.callback_speed_cmd)
        rospy.Subscriber('/angle2_cmd', Float32, self.callback_angle2_cmd)

        self.gyroz = 0

        self.angle_cmd = 0
        self.angle2_cmd = 0
        self.speed_cmd = 0


        # self.kps = [8,4]
        self.kps = [16,-16]
        # self.kps = [8,-4]

        # self.kps = [16,-8]
        # self.kis = [100,-100]

        self.kps = [16,-4]
        # self.kps = [.16,-.8]
        self.kis = [10,-5]
        # self.kis = [100,-100]

        self.esums = [0,0]
        self.mode = 0
        # self.kp = 10
        # self.kp = 50
        # self.kp = 250
        # self.kp = 1000

        # publisher for messages
        self.pub_angle = rospy.Publisher('angle_cmd', Int8, tcp_nodelay=True)
        
        self.spin()

if __name__ == '__main__':
    Node()