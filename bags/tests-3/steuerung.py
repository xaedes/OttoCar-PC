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


class Node(object):
    def latch(self,speed,angle,time):
        r = rospy.Rate(50) # 10hz
        starttime=self.time()
        while(self.run and (self.time()-starttime < time)):
            self.pub_angle.publish(angle) 
            self.pub_speed.publish(speed)
            r.sleep()

    def do(self,start,jump,speed,t_drive,t_halt,t_prehalt):
        self.latch(speed,start,t_drive)
        self.latch(speed,start+jump,t_drive)
        self.latch(-6,start,t_prehalt)
        self.latch(0,start,t_halt)
        self.latch(-speed,start,t_drive)
        self.latch(-speed,start+jump,t_drive)
        self.latch(1,start,t_prehalt)
        self.latch(0,start,t_halt)

    def spin(self):
        # r = rospy.Rate(50) # 10hz

        # for each speed there is a t_drive
        # speeds = [14,10,6]
        # t_drives = [0.6,1,2]   
        speeds = [8,10]
        t_drives = [2,0.75]   

        starts = [-20,-10,0,10,20]
        jumps = [1,8,20,40]

        # starts = [0]
        # jumps = [8]

        # t_halt = 0.5
        t_halt = 1
        t_prehalt = 0.25

        speeds_i = 0
        starts_i = 0
        jumps_i = 0

        estimated = 0
        for x in xrange(0,len(speeds)):
            estimated += (t_drives[x]*2+t_halt*2+t_prehalt*2)*2 * len(jumps)* len(starts)
        print "estimated time", estimated, "sec"
        print "estimated bag size", estimated * 1.1/27.7, "gb"

        while(self.run):
            t_drive=t_drives[speeds_i]
            self.do(starts[starts_i],jumps[jumps_i],speeds[speeds_i],t_drive,t_halt,t_prehalt)
            self.do(starts[starts_i],-jumps[jumps_i],speeds[speeds_i],t_drive,t_halt,t_prehalt)
            jumps_i += 1
            if(jumps_i>=len(jumps)):
                jumps_i=0
                starts_i += 1
                if(starts_i>=len(starts)):
                    starts_i=0
                    speeds_i += 1
                    if(speeds_i>=len(speeds)):
                        self.run=False


            # r.sleep()

    def time(self):
        return rospy.Time.now().to_sec()

    def __init__(self):
        super(Node, self).__init__()

        self.run = True
        self.pause = False
        signal.signal(signal.SIGINT, self.keyboard_interupt)  

        rospy.init_node('steuerung')

        # publisher for messages
        self.pub_angle = rospy.Publisher('angle_cmd', Int8, tcp_nodelay=True)
        self.pub_speed = rospy.Publisher('speed_cmd', Int8, tcp_nodelay=True)
        
        self.spin()

    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."
        sys.exit()


if __name__ == '__main__':
    Node()