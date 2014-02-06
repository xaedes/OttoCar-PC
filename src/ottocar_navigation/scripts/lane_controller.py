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
from std_msgs.msg import Float64MultiArray
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

class DTFilter(object):
    """docstring for DTFilter"""
    def __init__(self):
        super(DTFilter, self).__init__()

        self.last_time = None
        self.dt = None
        self.gain = 0.2

    def get_time(self):
        return time()
        
    def update(self):
        if(self.last_time == None):
            self.last_time = self.get_time()
            return
        
        time_now = self.get_time()
        dt_now = time_now - self.last_time
        self.last_time=time_now

        # wenn neuer wert ganz stark abweicht vom alten, schwaeche den gain factor ab
        confidence = pow(1./2000.,abs(dt_now - self.dt))
        self.dt = confidence * self.gain * dt_now + (1-confidence * self.gain) * self.dt
        


class Node(object):
    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."
        sys.exit()

    def spin(self):
        r = rospy.Rate(50) # in [hz]
        while(self.run):
            self.dt_filter.update()

            # 41cm = 57px
            target_angle = self.controller_target_angle.control(target=0, current=self.pos)
            yaw_cmd = self.controller_yaw.control(target=target_angle, current=self.angle)


            # angle_cmd = self.controller_angle.control(target=0.3, current=self.ir)
            # angle_cmd = self.controller_angle2.control(target=18, current=self.distance)
            # print(self.controller.esum,self.controller.esum)

            self.pub_yaw_cmd.publish(yaw_cmd)
            # self.pub_angle_cmd.publish(angle_cmd)

            r.sleep()

    def callback_orientation(self,msg):
        self.orientation = msg.data

    def callback_lanestate(self,msg):
        # layout:
        #   dim: []
        #   data_offset: 0
        # data: [946685062.8521258, 120.0, -0.0, 0.5]

        self.lanestate = msg.data
        self.pos = self.lanestate[1] * 41. / 57.                # in cm
        self.angle = self.lanestate[2] * 3.14159 / 180.         # in rad


        self.pub_dist.publish(self.distance)

    def __init__(self):
        super(Node, self).__init__()
        self.run = True
        self.pause = False
        signal.signal(signal.SIGINT, self.keyboard_interupt)  

        rospy.init_node('lane_controller')

        self.lanestate = None
        self.pos = None
        self.angle = None
        self.orientation = None

        self.dt_filter = DTFilter()

        self.controller_target_angle = Controller(kp=-.01, ki=0, esum_bounds=[-100,100], bounds=[-math.pi/8,+math.pi/8])
        # self.controller_target_angle = Controller(kp=.03, ki=0, esum_bounds=[-100,100], bounds=[-math.pi/8,+math.pi/8])
        self.controller_yaw = Controller(kp=-(1.5)/(math.pi/2.), ki=0, esum_bounds=[-100,100], bounds=[-1.5,+1.5])
        # self.controller_yaw = Controller(kp=5, ki=0, esum_bounds=[-100,100], bounds=[-1.5,+1.5])

        self.pub_yaw_cmd = rospy.Publisher('/yaw_rate_cmd', Float32, tcp_nodelay=True)
        # self.pub_angle_cmd = rospy.Publisher('/angle_cmd', Int8, tcp_nodelay=True)
        
        rospy.Subscriber('/ottocar_perception/laneState', Float64MultiArray, self.callback_lanestate)
        rospy.Subscriber('/orientation', Float32, self.callback_orientation)
        self.spin()

if __name__ == '__main__':
    Node()