#!/usr/bin/env python

import rospy
import sys

# performs a ros log with given command line arguments
# e.g. it can be used to log in launch files
# usage:
# rosrun ottocar_utils log.py type msg [arguments...]
#
#  type = debug|info|warn|err|fatal
#  arguments can be multiple arguments that are used in string formating (str.format) of msg
#
# example:
# rosrun ottocar_utils log.py info 'foo {} bar {} ananas' banana monkey
# [INFO] [WallTime: .................] foo banana bar monkey ananas


if __name__ == '__main__':
    rospy.init_node('log')
    if len(sys.argv) > 1:
        t = sys.argv[1]
        msg = sys.argv[2]
        args = sys.argv[3:]

        msg = msg.format(*args)

        if t == "error":
            t = "err"
        
        if t in ["debug","info","warn","err","fatal"]:
            log = getattr(rospy,"log%s" % t)
            log(msg)
        # rospy.logerr(str.format(sys.argv[2],*sys.argv[3:]))
    
