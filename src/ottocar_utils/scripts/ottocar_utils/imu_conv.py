#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

import sys

class Collector(object):
    """Collects data from imu and outputs it in different format"""


    def try_publish(self):
        if(self.accel and self.magnet and self.gyro):
            self.accel = self.magnet = self.gyro = False
            self.msg_imu.header.stamp = self.msg_mag.header.stamp = rospy.Time.now()
            self.pub_imu.publish(self.msg_imu)
            self.pub_mag.publish(self.msg_mag)


    def callback_accel(self, data):
        self.msg_imu.linear_acceleration.x = data.x
        self.msg_imu.linear_acceleration.y = data.y
        self.msg_imu.linear_acceleration.z = data.z
        self.accel = True
        self.try_publish()
        
    def callback_gyro(self, data):
        self.msg_imu.angular_velocity.x = data.x
        self.msg_imu.angular_velocity.y = data.y
        self.msg_imu.angular_velocity.z = data.z
        self.gyro = True
        self.try_publish()
        
    def callback_magnet(self, data):
        self.msg_mag.vector.x = data.x
        self.msg_mag.vector.y = data.y
        self.msg_mag.vector.z = data.z
        self.magnet = True
        self.try_publish()

    def callback_accel_var(self, data):
        self.msg_imu.linear_acceleration_covariance[0] = data.x
        self.msg_imu.linear_acceleration_covariance[4] = data.y
        self.msg_imu.linear_acceleration_covariance[8] = data.z
        self.subs_accel_var.unregister()


    def callback_gyro_var(self, data):
        self.msg_imu.angular_velocity_covariance[0] = data.x
        self.msg_imu.angular_velocity_covariance[4] = data.y
        self.msg_imu.angular_velocity_covariance[8] = data.z
        self.subs_gyro_var.unregister()



    def __init__(self):
        super(Collector, self).__init__()

        rospy.init_node('imu_conv', anonymous=True)

        # no values for accel, magnet and gyro available
        self.accel = False
        self.magnet = False
        self.gyro = False

        # create msg und publisher for output
        self.msg_imu = Imu()
        self.msg_mag = Vector3Stamped()
        self.pub_imu = rospy.Publisher('/imu/data_raw', Imu)
        self.pub_mag = rospy.Publisher('/imu/mag', Vector3Stamped)

        self.msg_imu.header.frame_id = self.msg_mag.header.frame_id = rospy.get_param('~frame_id','/imu')

        self.msg_imu.orientation.x = 0
        self.msg_imu.orientation.y = 0
        self.msg_imu.orientation.z = 0
        self.msg_imu.orientation.w = 0
        # set covariances zero
        for k in xrange(0,9):
            self.msg_imu.angular_velocity_covariance[k] = 0.0
            self.msg_imu.orientation_covariance[k] = 0.0
            self.msg_imu.linear_acceleration_covariance[k] = 0.0

        # set covariance along covariance
        for k in [0,4,8]:
            self.msg_imu.angular_velocity_covariance[k] = 1.0
            self.msg_imu.orientation_covariance[k] = 1.0
            self.msg_imu.linear_acceleration_covariance[k] = 1.0

        rospy.sleep(1)
        # subscribe to imu data
        rospy.Subscriber('/accelerometer/raw', Vector3, self.callback_accel)
        rospy.Subscriber('/magnetometer/raw', Vector3, self.callback_magnet)
        rospy.Subscriber('/gyroscope/calibrated', Vector3, self.callback_gyro)
        self.subs_accel_var = rospy.Subscriber('/accelerometer/variances', Vector3, self.callback_accel_var)
        # self.subs_magnet_var = rospy.Subscriber(rospy.get_param('~topic_in_magnetometer_variances'), Vector3, self.callback_magnet_var)
        self.subs_gyro_var = rospy.Subscriber('/gyroscope/variances', Vector3, self.callback_gyro_var)
        rospy.spin()




if __name__ == '__main__':
    try:
        collector = Collector()



    except rospy.ROSInterruptException:
        pass


