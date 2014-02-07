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
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

class Controller:
    def control(self, target, current):
        self.dt = time() - self.last_time
        self.last_time = time()
        e = target - current
        self.last_e = e
        self.esum += e
        self.esum = min(max(self.esum, self.esum_bounds[0]), self.esum_bounds[1])
        
        output = self.zero + e * self.kp + self.ki * self.dt * self.esum
        # output = self.last + e * self.kp + self.ki * self.dt * self.esum
        output = min(max(output, self.bounds[0]), self.bounds[1])

        self.last = output

        # print e

        return output

    def __init__(self, kp, ki, esum_bounds, bounds, zero=0):
        self.zero = 0
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
        self.dt = 0
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
            # print self.

            if((self.pos!=None)and(self.angle!=None)):

                # if(self.orientation != None):
                #     # interpolate between lanestate msgs
                #     if(self.orientation_anchor == None):
                #         self.orientation_anchor = self.orientation
                        
                #     angle = self.angle + (self.orientation - self.orientation_anchor) * 180 / pi
                # else:
                angle = self.angle

                self.pub_dbg_angle.publish(angle)
                # 41cm = 57px

                # step = 1

                # if(step==1):
                angle_cmd_pos = self.controller_pos_with_angle_cmd.control(target=20, current=self.pos)
                angle_cmd_angle = self.controller_angle_with_angle_cmd.control(target=0, current=angle)

                angle_cmd = angle_cmd_pos + angle_cmd_angle
                # angle_cmd = angle_cmd_pos + angle_cmd_angle
                print angle_cmd_pos, angle_cmd_angle, angle_cmd

                # target_angle = self.controller_target_angle.control(target=0, current=self.pos)
                # angle_cmd = self.controller_angle.control(target=target_angle, current=self.angle)
                    # yaw_cmd = self.controller_yaw.control(target=0, current=self.angle)
                # elif(step==2):
                #     target_angle = self.controller_target_angle.control(target=0, current=self.pos)
                #     angle_cmd = self.controller_angle.control(target=target_angle, current=self.angle)
                #     # yaw_cmd = self.controller_yaw.control(target=target_angle, current=self.angle)


                # angle_cmd = self.controller_angle.control(target=0.3, current=self.ir)
                # angle_cmd = self.controller_angle2.control(target=18, current=self.distance)
                # print(self.controller.esum,self.controller.esum)
                # print self.lanestate[0]-rospy.Time.now().to_sec(),self.lanestate[0],rospy.Time.now().to_sec()
                # print self.controller_pos_with_angle_cmd.last_e, angle_cmd 
                # print self.controller_target_angle.last_e, target_angle*10 #, self.controller_angle.last_e, angle_cmd
                # self.pub_yaw_cmd.publish(yaw_cmd)
                self.pub_angle_cmd.publish(int(min(max(angle_cmd,-126),126)))
                # self.pub_angle_cmd.publish(int(angle_cmd))

            r.sleep()

    def callback_orientation(self,msg):
        self.orientation = msg.data


    def callback_lanestate(self,msg):
        # layout:
        #   dim: []
        #   data_offset: 0
        # data: [946685062.8521258, 120.0, -0.0, 0.5]
        print msg
        self.lanestate = msg.data
        self.pos = self.lanestate[1]           # in cm
        self.angle = self.lanestate[2]         # in grad
        # self.angle = self.lanestate[2] * 3.14159 / 180.         # in rad

        self.orientation_anchor = self.orientation


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
        self.orientation_anchor = None

        self.dt_filter = DTFilter()

        self.controller_pos_with_angle_cmd = Controller(kp=5, ki=0, esum_bounds=[-100,100], bounds=[-126.,+126.])
        self.controller_angle_with_angle_cmd = Controller(kp=-10, ki=0, esum_bounds=[-100,100], bounds=[-126.,+126.])

        # self.controller_target_angle = Controller(kp=-0.01, ki=0, esum_bounds=[-100,100], bounds=[-11.,+11.])
        # self.controller_target_angle = Controller(kp=-25, ki=0, esum_bounds=[-100,100], bounds=[-math.pi/16,+math.pi/16])
        # self.controller_target_angle = Controller(kp=-.01, ki=0, esum_bounds=[-100,100], bounds=[-math.pi/8,+math.pi/8])
        # self.controller_target_angle = Controller(kp=.03, ki=0, esum_bounds=[-100,100], bounds=[-math.pi/8,+math.pi/8])
        # self.controller_yaw = Controller(kp=(1.5)/(math.pi/2.), ki=0, esum_bounds=[-100,100], bounds=[-1.5,+1.5])
        # self.controller_yaw = Controller(kp=(1.5)/(math.pi/2.), ki=0, esum_bounds=[-100,100], bounds=[-1.5,+1.5])
        # self.controller_yaw = Controller(kp=-5, ki=0, esum_bounds=[-100,100], bounds=[-3,+3])
        # self.controller_angle = Controller(kp=-10, ki=0, esum_bounds=[-100,100], bounds=[-126,+126])

        self.pub_yaw_cmd = rospy.Publisher('/yaw_rate_cmd', Float32, tcp_nodelay=True)
        self.pub_angle_cmd = rospy.Publisher('/angle_cmd', Int8, tcp_nodelay=True)
        self.pub_dbg_angle = rospy.Publisher('/dbg/angle', Float32, tcp_nodelay=True)
        
        rospy.Subscriber('/ottocar_perception/laneState', Float64MultiArray, self.callback_lanestate)
        rospy.Subscriber('/orientation', Float32, self.callback_orientation)
        self.spin()

if __name__ == '__main__':
    Node()