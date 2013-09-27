#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

# make sure this file is executable
# make sure roscore is running
# to run this script : rosrun allan_variance listener.py

def calc(data):
	max_t = len(data) // 9

	deviations = np.empty(shape=(max_t));


	def avar(np_data,t):
		# split data into bins of length t ; most likely there will be too much elements for a perfect split, throw these away
		# average data in each bin to obtain a list of averages
		# calculate allan variance from the averages
		n_bins = np_data.shape[0] // t 
		means = np_data[0:(n_bins*t)].reshape((n_bins,t)).mean(axis=1)
		return np.sum((means[0:-1] - means[1:]) ** 2) / (2*(n_bins-1))

	np_data = np.array(data)
	for t in xrange(1,max_t+1):
		deviations[t-1] = sqrt(avar(np_data,t))
	
	# plt.ion()
	# plt.clf()
	plt.loglog(deviations)
	# plt.draw()
	plt.show()
	# plt.show(block=False)
	# return

def listener():

	collected_data_x = []
	collected_data_y = []
	collected_data_z = []



	def callback(data):
		collected_data_x.append(data.x)
		collected_data_y.append(data.y)
		collected_data_z.append(data.z)
		# if(len(collected_data_x)%1):
			# rospy.loginfo(rospy.get_name() + "collected: %d" % len(collected_data_x))

		if(len(collected_data_x) == 213528):
			rospy.loginfo(rospy.get_name() + "collected: %d" % len(collected_data_x))
			calc(collected_data_x)

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("accelerometer", Vector3, callback)
	rospy.spin()


if __name__ == '__main__':
    listener()