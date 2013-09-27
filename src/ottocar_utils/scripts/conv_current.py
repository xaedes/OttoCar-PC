#!/usr/bin/env python
import rospy

from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import Float32

class Node(object):
	"""Converts /current output of uC to correct value"""

	def callback_speed_cmd(self, data):
		self.speed = data.data

	def callback_current(self, data):
		speed_max = 127
		duty_cycle = abs(self.speed*1.0) / speed_max	
		self.pub.publish( data.data * 2.56 * duty_cycle / (1024 * (1+(100/3.9)) * 0.005)  )

	def __init__(self):
		super(Node, self).__init__()
		self.speed = 0
		rospy.init_node('foo', anonymous=True)
		self.pub = rospy.Publisher('/current2', Float32)
		rospy.Subscriber('/current', Int16, self.callback_current)
		rospy.Subscriber('/speed_cmd', Int8, self.callback_speed_cmd)
		rospy.spin()

		

if __name__ == '__main__':
	node = Node()