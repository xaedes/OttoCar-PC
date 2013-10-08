#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
# from std_msgs.msg import Boolean

def Vector3_to_nparray(vec):
    return np.array([vec.x,vex.y,vec.z])

def nparray_to_Vector3(arr):
    return Vector3(arr[0,0],arr[1,0],arr[2,0])

class CalibrateVector3(object):
    """docstring for CalibrateVector3"""
    def __init__(self, stop_after,topic_in, topic_calibrated, topic_variances):
        super(CalibrateVector3, self).__init__()

        self.stop_after = stop_after
        self.reset_calibration()
        self.start_calibration()

        rospy.Subscriber(topic_in, Vector3, self.callback)
        self.publisher_calibrated = rospy.Publisher(topic_calibrated, Vector3)
        self.publisher_variances = rospy.Publisher(topic_variances, Vector3)


    def callback(self, vec):
        if self.calibrate:
            self.data.append([vec.x,vec.y,vec.z])
            if (len(self.data) == self.stop_after):
                self.stop_calibration()
        else:
            # print 'self.mean.shape', self.mean.shape

            calibrated = Vector3()
            calibrated.x = vec.x - self.mean[0]
            calibrated.y = vec.y - self.mean[1]
            calibrated.z = vec.z - self.mean[2]

            self.publisher_calibrated.publish(calibrated)

            variances = Vector3()
            variances.x = self.std[0]
            variances.y = self.std[1]
            variances.z = self.std[2]
            self.publisher_variances.publish(variances)

    def start_calibration(self):
        self.calibrate = True

    def stop_calibration(self):
        self.calibrate = False
        self.data_array = np.array(self.data)
        self.mean = self.data_array.mean(axis=0)
        self.std = self.data_array.std(axis=0)

    def reset_calibration(self):
        self.data = []
        self.calibrate = False

if __name__ == '__main__':
    rospy.init_node('calibration_filter', anonymous=True)
    calib = CalibrateVector3(rospy.get_param('~n_samples',100), 
                             '/in', 
                             '/calibrated', 
                             '/variances')
    rospy.spin()


