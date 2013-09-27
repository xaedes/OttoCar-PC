#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

import sys


# def callback(data):
#     rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)


# def talker():
#     pub = rospy.Publisher('chatter', String)
#     rospy.init_node('talker')
#     while not rospy.is_shutdown():
#         str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(str)
#         pub.publish(String(str))
#         rospy.sleep(1.0)

# def listener():

class Collector(object):
    """Collects data from imu and outputs it in different format"""


    def try_publish(self):
        if(self.accel and self.magnet and self.gyro):
            self.accel = self.magnet = self.gyro = False
            self.pub.publish(self.msg)


    def callback_accel(self, data):
        self.msg.linear_acceleration.x = data.x
        self.msg.linear_acceleration.y = data.y
        self.msg.linear_acceleration.z = data.z
        self.accel = True
        self.try_publish()
        
    def callback_gyro(self, data):
        self.msg.angular_velocity.x = data.x
        self.msg.angular_velocity.y = data.y
        self.msg.angular_velocity.z = data.z
        self.gyro = True
        self.try_publish()
        
    def callback_magnet(self, data):
        self.msg.orientation.x = data.x
        self.msg.orientation.y = data.y
        self.msg.orientation.z = data.z
        self.msg.orientation.w = 1
        self.magnet = True
        self.try_publish()

    def __init__(self):
        super(Collector, self).__init__()

        rospy.init_node('imu_conv', anonymous=True)

        self.accel = False
        self.magnet = False
        self.gyro = False
        self.msg = Imu()
        self.pub = rospy.Publisher('imu_data', Imu)

        for k in xrange(0,9):
        	self.msg.angular_velocity_covariance[k] = 0.1
        	self.msg.orientation_covariance[k] = 0.1
        	self.msg.linear_acceleration_covariance[k] = 0.1

        rospy.Subscriber('accelerometer', Vector3, self.callback_accel)
        rospy.Subscriber('magnetometer', Vector3, self.callback_magnet)
        rospy.Subscriber('gyroscope', Vector3, self.callback_gyro)
        rospy.spin()




if __name__ == '__main__':
    try:
        collector = Collector()
        



    except rospy.ROSInterruptException:
        pass


