#!/usr/bin/env python

import rospy
import sys

class Node(object):
    def __init__(self):
        super(Node, self).__init__()
        rospy.init_node('a')
        if len(sys.argv) > 1:
            self.guard_name = sys.argv[1]
            rospy.set_param(self.guard_name,"True")
        rospy.spin()

    def __del__(self):
        if hasattr(self,"guard_name") and rospy.has_param(self.guard_name):
            rospy.delete_param(self.guard_name)

if __name__ == '__main__':
    node = Node()