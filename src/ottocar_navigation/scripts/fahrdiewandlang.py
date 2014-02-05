#!/usr/bin/env python


from time import sleep, time


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

class Controller:
    def control(self, target, current):
        self.dt = time() - self.last_time
        self.last_time = time()
        e = target - current
        self.esum += e
        self.esum = min(max(self.esum, self.esum_bounds[0]), self.esum_bounds[1])
        
        output = self.last + e * self.kp + self.ki * self.dt * self.esum
        output = min(max(output, self.bounds[0]), self.bounds[1])

        self.last = output

        # print e

        return output

    def __init__(self, kp, ki, esum_bounds, bounds):
        self.dt = 0
        self.kp = kp
        self.ki = ki
        self.esum = 0
        self.esum_bounds = esum_bounds
        self.bounds = bounds
        self.last = 0
        self.last_time = time()


class Node(object):
    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."
        sys.exit()

    def spin(self):
        r = rospy.Rate(50) # in [hz]
        while(self.run):

            if(self.ir==None):
                continue

            # yaw_cmd = self.controller_yaw.control(target=0.8, current=self.ir)
            # angle_cmd = self.controller_angle.control(target=0.3, current=self.ir)
            angle_cmd = self.controller_angle2.control(target=18, current=self.distance)
            # print(self.controller.esum,self.controller.esum)

            # self.pub_yaw_cmd.publish(yaw_cmd)
            self.pub_angle_cmd.publish(angle_cmd)

            r.sleep()

    def callback_ir(self,data):
        self.ir = data.data


        # http://www.rn-wissen.de/index.php/Sensorarten#Formel_zur_Entfernungsberechnung
        d1=8.
        x1=1.48
        d2=25
        x2=0.47

        A = ((x2 - x1) * d2 * d1) / (d1 - d2 ) 
        B = (d2 * x2 - d1 * x1) / (d2 - d1)

        # print A, B

        self.distance = (A / (self.ir - B))


        self.pub_dist.publish(self.distance)

    def __init__(self):
        super(Node, self).__init__()
        self.run = True
        self.pause = False
        signal.signal(signal.SIGINT, self.keyboard_interupt)  

        rospy.init_node('fahrdiewandlang')


        rospy.Subscriber('/sensor_IR2', Float32, self.callback_ir)
        self.ir=None

        # self.controller_yaw = Controller(kp=.056, ki=0, esum_bounds=[-100,100], bounds=[-1.5,+1.5])
        # self.controller_angle = Controller(kp=10000, ki=0.1, esum_bounds=[-100,100], bounds=[-100*0.46,+80*0.46])
        self.controller_angle2 = Controller(kp=-40, ki=0.1, esum_bounds=[-100,100], bounds=[-100*0.46,+80*0.46])


        self.pub_yaw_cmd = rospy.Publisher('/yaw_rate_cmd', Float32, tcp_nodelay=True)
        self.pub_angle_cmd = rospy.Publisher('/angle_cmd', Int8, tcp_nodelay=True)
        self.pub_dist = rospy.Publisher('/dist', Float32, tcp_nodelay=True)
        
        self.spin()

if __name__ == '__main__':
    Node()