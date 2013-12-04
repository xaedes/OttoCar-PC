#!/usr/bin/env python

from os.path import basename

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


class Node(object):
    """Subscribes to imu data with orientation and removes gravity by determining gravity with a low pass filter over the acceleration."""

    def __init__(self, ):
        super(Node, self).__init__()
        rospy.init_node(basename(__file__).replace('.','_'))


        self.imu_data = Imu()
        self.low_pass = Vector3()
        self.raw = Vector3()

        rospy.Subscriber('/imu/data', Imu, self.callback_imu)
        rospy.Subscriber('/accelerometer/raw', Vector3, self.callback_raw)
        rospy.Subscriber('/accelerometer/low_pass', Vector3, self.callback_low_pass)
        self.pub_imu = rospy.Publisher('/imu/data_wo_gravity', Imu)


        rospy.spin()

    def callback_low_pass(self, data):
        self.low_pass = data
    def callback_raw(self, data):
        self.raw = data

    def callback_imu(self, data):
        self.imu_data = data

        # 
        self.imu_data.linear_acceleration = self.raw
        self.imu_data.linear_acceleration.x -= self.low_pass.x
        self.imu_data.linear_acceleration.y -= self.low_pass.y
        self.imu_data.linear_acceleration.z -= self.low_pass.z

        self.pub_imu.publish(self.imu_data)
        


if __name__ == '__main__':
    Node()