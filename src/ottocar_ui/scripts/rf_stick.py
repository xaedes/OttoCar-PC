#!/usr/bin/env python2
import serial
import threading
from time import *
import struct
import rospy
import sys
from sensor_msgs.msg import Joy
 
class serialNode(object):
    def __init__(self):  
        self.baud = 115200
        init=0
        while init==0 and self.run:
            try:
                self.ser = serial.Serial('/dev/ttyACM0', self.baud, timeout=None, stopbits=serial.STOPBITS_ONE)
                init=1
            except serial.serialutil.SerialException as e:
                print "Waiting for connetion.."
                # print "Unexpected error:", sys.exc_info()[0]
                print e
                sleep(1)
           
        rospy.init_node('Remote_node', anonymous=True)
        self.pub_angle = rospy.Publisher('/joy', Joy, tcp_nodelay=True)
       
       
    def checksum(self, list):
        sum=0
        for objects in range(0,len(list)-1):
            sum = sum + list[objects]
        return sum%256
       
   
   
 
    def run(self):
        blub=Joy()
        while(self.run):
            data=ord(self.ser.read(1))
            if data==255:
                data=ord(self.ser.read(1))
                if data==255:
                    data=ord(self.ser.read(1))
                    if data==255:
                        data=ord(self.ser.read(1))
                        if data==24:
                            data=self.ser.read(7)
                            axes = struct.unpack('BBBBBBB', data)
                            for x in axes:
                                blub.axes.append((x-128)/127.0)
                            print blub.axes
                            data=self.ser.read(2)
                            button = struct.unpack('H', data)[0]
                            blub.buttons.insert(0,int(button/2048))
                            button-=(int(button/2048))*button
                            blub.buttons.insert(0,int(button/1024))
                            button-=(int(button/1024))*button
                            blub.buttons.insert(0,int(button/512))
                            button-=(int(button/512))*button
                            blub.buttons.insert(0,int(button/256))
                            button-=(int(button/256))*button
                            blub.buttons.insert(0,int(button/128))
                            button-=(int(button/128))*button
                            blub.buttons.insert(0,int(button/64))
                            button-=(int(button/64))*button
                            blub.buttons.insert(0,int(button/32))
                            button-=(int(button/32))*button
                            blub.buttons.insert(0,int(button/16))
                            button-=(int(button/16))*button
                            blub.buttons.insert(0,int(button/8))
                            button-=(int(button/8))*button
                            blub.buttons.insert(0,int(button/4))
                            button-=(int(button/4))*button
                            blub.buttons.insert(0,int(button/2))
                            button-=(int(button/2))*button
                            blub.buttons.insert(0,int(button))
                            print blub
                            blub.axes=list()
                            blub.buttons=list()
                           
        self.ser.close()
       
 
if __name__ == '__main__':
    process=serialNode( )
    process.run()

