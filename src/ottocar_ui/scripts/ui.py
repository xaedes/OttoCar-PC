#!/usr/bin/env python

# import roslib; roslib.load_manifest('example')
import rospy

from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int16

from PySide.QtCore import * 
from PySide.QtGui import * 
import sys

class GUI:
    def __init__(self): 
        # A publisher for messages
        self.pub_angle = rospy.Publisher('angle_cmd', Int16)
        self.pub_speed = rospy.Publisher('speed_cmd', Int8)

        # A subscriber for our own messages
        self.sub = rospy.Subscriber('example', String, self.sub_callback)

        self.button = QPushButton('push\nme')

        # self.button.clicked.connect(self.pub_callback)

        self.sliderAngle = QSlider()
        self.sliderAngle.setMinimum(250)
        self.sliderAngle.setMaximum(500)
        self.sliderAngle.setSingleStep(1)

        self.sliderAngle.valueChanged.connect(self.sliderAngleValueChanged)

        self.sliderSpeed = QSlider()
        self.sliderSpeed.setMinimum(-20)
        self.sliderSpeed.setMaximum(20)
        self.sliderSpeed.setSingleStep(1)

        self.sliderSpeed.valueChanged.connect(self.sliderSpeedValueChanged)

        self.sliderSpeed.show()
        self.sliderAngle.show()
        # self.button.show()

    def sliderAngleValueChanged(self, chg):
        self.pub_angle.publish(chg)
        print 'angle: ', chg

    def sliderSpeedValueChanged(self, chg):
        self.pub_speed.publish(chg)
        print 'speed: ', chg

    def sub_callback(self, msg):
        print 'Got message:', msg.data


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('example')

    # Initialize Qt
    app = QApplication(sys.argv)

    gui = GUI()

    app.exec_()