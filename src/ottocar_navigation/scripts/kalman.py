#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

import threading

class Kalman(object):
	"""docstring for Kalman"""
	# http://www.cbcity.de/das-kalman-filter-einfach-erklaert-teil-2
	#http://de.wikipedia.org/wiki/Kalman-Filter
	def __init__(self, n_states, n_sensors):
		super(Kalman, self).__init__()
		self.n_states = n_states
		self.n_sensors = n_sensors

		# x: Systemzustand
		self.x = np.matrix(np.zeros(shape=(n_states,1)))

		# P: Unsicherheit ueber Systemzustand
		self.P = np.matrix(np.identity(n_states)) 

		# F: Dynamik
		self.F = np.matrix(np.identity(n_states))

		# Q: Dynamik Unsicherheit
		self.Q = np.matrix(np.zeros(shape=(n_states,n_states)))

		# u: externe Beeinflussung des Systems
		self.u = np.matrix(np.zeros(shape=(n_states,1)))

		# B: Dynamik der externen Einfluesse
		self.B = np.matrix(np.identity(n_states))

		# H: Messmatrix
		self.H = np.matrix(np.zeros(shape=(n_sensors, n_states)))

		# R: Messunsicherheit
		self.R = np.matrix(np.identity(n_sensors))

		# I: Einheitsmatrix
		self.I = np.matrix(np.identity(n_states))

		self.first = True

	def update(self, Z):
		'''Z: new sensor values as numpy matrix'''

		# print 'Z.shape', Z.shape
		# print 'self.H.shape', self.H.shape
		# print 'self.x.shape', self.x.shape

		# w: Innovation
		w = Z - self.H * self.x

		# S: Residualkovarianz (http://de.wikipedia.org/wiki/Kalman-Filter#Korrektur)
		S = self.H * self.P * self.H.getT() + self.R 		# sieht in wikipedia etwas anders aus..

		# K: Kalman-Gain
		K = self.P * self.H.getT() * S.getI()

		# x: Systemzustand
		self.x = self.x + K * w

		# P: Unsicherheit der Dynamik
		self.P = (self.I - K * self.H) * self.P

	def predict(self):
		self.x = self.F * self.x + self.B * self.u 	# de.wikipedia hat noch ein B vor das u multipliziert, B scheint die Dynamik der Stoerung u zu sein
		self.P = self.F * self.P * self.F.getT() + self.Q	# de.wikipedia hat noch ein Q drauf addiert

class Subscriber(object):
	"""docstring for Subscriber"""
	def __init__(self):
		super(Subscriber, self).__init__()
		rospy.init_node('kalman', anonymous=True)

		self.dt = dt = 1

		#states:
		# [accel.x,accel.y,accel.z,		0:3
		#  gyro.x,gyro.y,gyro.z,		3:6
		#  mag.x,mag.y,mag.z			6:9
		#  ]					

		self.kalman = Kalman(n_states = 9, n_sensors = 9)


		# H: Messmatrix
		self.kalman.H = np.matrix(	'1 0 0 0 0 0 0 0 0;'	#accel.x
									'0 1 0 0 0 0 0 0 0;'	#accel.y
									'0 0 1 0 0 0 0 0 0;'	#accel.z
									'0 0 0 1 0 0 0 0 0;'	#gyro.x
									'0 0 0 0 1 0 0 0 0;'	#gyro.y
									'0 0 0 0 0 1 0 0 0;'	#gyro.z
									'0 0 0 0 0 0 1 0 0;'	#mag.x
									'0 0 0 0 0 0 0 1 0;'	#mag.y
									'0 0 0 0 0 0 0 0 1'		#mag.z
			
									)
		# F: Dynamik
		self.kalman.F = np.matrix([	[1,0,0,0,0,0,0,0,0],	#accel.x = accel.x
									[0,1,0,0,0,0,0,0,0],	#accel.y = accel.y
									[0,0,1,0,0,0,0,0,0],	#accel.z = accel.z
									[0,0,0,1,0,0,0,0,0],	#gyro.x = gyro.x
									[0,0,0,0,1,0,0,0,0],	#gyro.y = gyro.y
									[0,0,0,0,0,1,0,0,0],	#gyro.z = gyro.z
									[0,0,0,0,0,0,1,0,0],	#mag.x = mag.x
									[0,0,0,0,0,0,0,1,0],	#mag.y = mag.y
									[0,0,0,0,0,0,0,0,1] 	#mag.z = mag.z
									])

		# Q: Unsicherheit der Dynamik 
		self.kalman.Q = np.matrix(np.identity(self.kalman.n_states)) * 0.1


		# P: Unsicherheit ueber Systemzustand	
		self.P = 0.1
		self.kalman.P *= self.P

		# R: Messunsicherheit
		# self.kalman.R *= 1000 ** 3
		self.kalman.R *= 1
		# self.kalman.R *= 0.000001

		# Publishers
		self.pub_imu = rospy.Publisher('/imu/filtered', Imu)
		self.pub_mag = rospy.Publisher('/imu/mag_filtered', Vector3Stamped)

		# Subscribers
		rospy.Subscriber('/imu/data_raw', Imu, self.callback_imu)
		rospy.Subscriber('/imu/mag', Vector3Stamped, self.callback_mag)

		self.imu = self.mag = None
		self.lock = threading.Lock()

		rospy.spin()

	def callback_imu(self, msg):
		self.imu = msg

		self.lock.acquire()
		self.measure()
		self.lock.release()

	def callback_mag(self, msg):
		self.mag = msg

		self.lock.acquire()
		self.measure()
		self.lock.release()

	def measure(self):
		# only proceed if we have all msgs
		if((self.mag==None)or(self.imu==None)):
			return


		Z = np.matrix([self.imu.linear_acceleration.x,self.imu.linear_acceleration.y,self.imu.linear_acceleration.z,
						self.imu.angular_velocity.x,self.imu.angular_velocity.y,self.imu.angular_velocity.z,
						self.mag.vector.x,self.mag.vector.y,self.mag.vector.z]).getT()

		if self.kalman.first:
			self.kalman.first = False

		self.kalman.update(Z)
		self.kalman.predict()

		header = self.mag.header

		# publish filtered data
		imu = Imu()
		imu.header = header
		imu.linear_acceleration.x = self.kalman.x[0]
		imu.linear_acceleration.y = self.kalman.x[1]
		imu.linear_acceleration.z = self.kalman.x[2]
		imu.angular_velocity.x = self.kalman.x[3]
		imu.angular_velocity.y = self.kalman.x[4]
		imu.angular_velocity.z = self.kalman.x[5]
		self.pub_imu.publish(imu)

		mag = Vector3Stamped()
		mag.header = header
		mag.vector.x = self.kalman.x[6]
		mag.vector.y = self.kalman.x[7]
		mag.vector.z = self.kalman.x[8]
		self.pub_mag.publish(mag)

		# remove old msgs
		self.imu = None
		self.mag = None

		


if __name__ == '__main__':
	subscriber = Subscriber()


