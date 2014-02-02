#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Vector3

class Kalman(object):
	"""docstring for Kalman"""
	# http://www.cbcity.de/das-kalman-filter-einfach-erklaert-teil-2
	def __init__(self, n_states, n_sensors):
		super(Kalman, self).__init__()
		self.n_states = n_states
		self.n_sensors = n_sensors

		# x: Systemzustand
		self.x = np.matrix(np.zeros(shape=(n_states,1)))

		# P: Unsicherheit über Systemzustand
		self.P = np.matrix(np.identity(n_states)) 

		# F: Dynamik
		self.F = np.matrix(np.identity(n_states))

		# u: externe Beeinflussung des Systems
		self.u = np.matrix(np.zeros(shape=(n_states,1)))

		# H: Messmatrix
		self.H = np.matrix(np.zeros(shape=(n_sensors, n_states)))

		# R: Messunsicherheit
		self.R = np.matrix(np.identity(n_sensors))

		# I: Identity
		self.I = np.matrix(np.identity(n_states))

		self.first = True

	def update(self, Z):
		'''Z: new sensor values as numpy matrix'''

		# print 'Z.shape', Z.shape
		# print 'self.H.shape', self.H.shape
		# print 'self.x.shape', self.x.shape

		w = Z - self.H * self.x
		S = self.H * self.P * self.H.getT() + self.R
		K = self.P * self.H.getT() * S.getI()
		self.x = self.x + K * w
		self.P = (self.I - K * self.H) * self.P

	def predict(self):
		self.x = self.F * self.x + self.u
		self.P = self.F * self.P * self.F.getT()

class Subscriber(object):
	"""docstring for Subscriber"""
	def __init__(self):
		super(Subscriber, self).__init__()
		rospy.init_node('imu_conv', anonymous=True)

		self.dt = dt = 1

		self.kalman = Kalman(n_states = 9, n_sensors = 5)
		self.kalman.H = np.matrix(	'0 0 0 0 0 0 1 0 0;'	#accel.x
									'0 0 0 0 0 0 0 1 0;'	#accel.y
									'0 0 0 0 0 0 0 0 1;'	#accel.z
									'0 0 0 0 0 1 0 0 0;'	#assume: veloc.z = 0
									'0 0 1 0 0 0 0 0 0 '	#assume: pos.z = 0
									)
		self.kalman.F = np.matrix([	[1,0,0,dt,0,0,0,0,0],	#pos.x = pos.x + dt*veloc.x
									[0,1,0,0,dt,0,0,0,0],	#pos.y = pos.y + dt*veloc.y
									[0,0,1,0,0,dt,0,0,0],	#pos.y = pos.z + dt*veloc.z
									[0,0,0,1,0,0,dt,0,0],	#veloc.x = veloc.x + dt*accel.x
									[0,0,0,0,1,0,0,dt,0],	#veloc.y = veloc.y + dt*accel.y
									[0,0,0,0,0,1,0,0,dt],	#veloc.z = veloc.z + dt*accel.z
									[0,0,0,0,0,0,1,0,0],	#accel.x = accel.x
									[0,0,0,0,0,0,0,1,0],	#accel.y = accel.y
									[0,0,0,0,0,0,0,0,1] 	#accel.z = accel.z
									])



		# self.kalman = Kalman(n_states = 3, n_sensors = 3)
		# self.kalman.H = np.matrix(	'1 0 0;'
		# 							'0 1 0;'
		# 							'0 0 1')

		self.P = 0.1

		self.kalman.P *= self.P
		# self.kalman.R *= 1000 ** 3
		self.kalman.R *= 1
		# self.kalman.R *= 0.000000000000000000000000000000000000002

		self.pub_accel = rospy.Publisher('/accelerometer/kalman', Vector3)
		self.pub_veloc = rospy.Publisher('/velocity/kalman', Vector3)
		self.pub_pos = rospy.Publisher('/position/kalman', Vector3)
		# self.pub_accel_var_min = rospy.Publisher('kalman/accelerometer_var_min', Vector3)
		# self.pub_accel_var_max = rospy.Publisher('kalman/accelerometer_var_max', Vector3)

		rospy.Subscriber('/accelerometer/calibrated', Vector3, self.callback_accel)
		rospy.spin()
		
	def callback_accel(self, data):
		# print "received data: ", data
		Z = np.matrix([data.x,data.y,data.z,0,0]).getT()

		if self.kalman.first:
			# self.kalman.x[3:6,0] = Z
			self.kalman.first = False

		self.kalman.P[6:9,6:9] = np.matrix(np.identity(3)) * self.P

		# print self.kalman.P

		self.kalman.update(Z)
		self.kalman.predict()

		vec = Vector3()
		vec.x = self.kalman.x[0]
		vec.y = self.kalman.x[1]
		vec.z = self.kalman.x[2]

		self.pub_pos.publish(vec)

		vec = Vector3()
		vec.x = self.kalman.x[3]
		vec.y = self.kalman.x[4]
		vec.z = self.kalman.x[5]

		self.pub_veloc.publish(vec)

		vec = Vector3()
		vec.x = self.kalman.x[6]
		vec.y = self.kalman.x[7]
		vec.z = self.kalman.x[8]

		self.pub_accel.publish(vec)

		# foo = 100
		# vec = Vector3()
		# vec.x = self.kalman.x[0,0] - self.kalman.P[0,0] * foo
		# vec.y = self.kalman.x[1,0] - self.kalman.P[1,1] * foo
		# vec.z = self.kalman.x[2,0] - self.kalman.P[2,2] * foo

		# self.pub_accel_var_min.publish(vec)

		# vec = Vector3()
		# vec.x = self.kalman.x[0,0] + self.kalman.P[0,0] * foo
		# vec.y = self.kalman.x[1,0] + self.kalman.P[1,1] * foo
		# vec.z = self.kalman.x[2,0] + self.kalman.P[2,2] * foo

		# self.pub_accel_var_max.publish(vec)



if __name__ == '__main__':
	subscriber = Subscriber()


