#!/usr/bin/env python

import rospy
import numpy as np
import math
import signal
import sys
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32

class TrajectoryController(object):
    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing..."
        sys.exit()

    def resample(self, line, interval):
        differences = line[1:] - line[:-1]
        lens = np.sqrt((differences*differences).sum(axis=1)) # in cm
        cumsum_lens = np.cumsum(lens)

        # print line.shape,differences.shape

        # returns point on line at (linear) position l 
        def line_fun(l):
            k = 0
            while k<n:
                if l < lens[k]:
                    # section found
                    # linear interpolate between section start and end
                    return tuple(line[k] + differences[k] * (l/lens[k]))
                print k, l, lens[k],
                l -= lens[k]
                print l
                k += 1

            # advance line
            return tuple(line[-1,:] + differences[-1] * (l/lens[-1]))

        # resample line
        interval=10. # in cm
        n_resampled = int(math.ceil(cumsum_lens[-1] / interval))

        resampled = np.zeros(shape=(n_resampled,2))
        for k in xrange(0,n_resampled):
            resampled[k,:] = line_fun(k*interval)

        return resampled

    def callback_mid_lane(self, msg):
        n = len(msg.data)/2
        if n == 1:
            print "only one point given, really??? i expected a line, which needs more than one point.... meh"
            print "msg", msg
            return



        line = np.zeros(shape=(n,2))
        for k in xrange(0,line.shape[0]):
            line[k] = msg.data[k*2], msg.data[k*2 + 1]

        interval = 10.  # in cm
        # resample line
        self.resampled = self.resample(line,interval)

        # get segment vectors
        self.diffs = self.resampled[1:] - self.resampled[:-1]

        # length of segment vectors (roughly the same as interval)
        # self.lens[0] - len of first segment
        self.lens = np.sqrt((self.diffs*self.diffs).sum(axis=1)) # in cm

        # cummulative lengths
        self.cumsum_lens = np.cumsum(self.lens)

        # angles of segments
        self.angles = np.arctan2(self.diffs[:,0],self.diffs[:,1])

        # angles of tangents at segment joints
        self.tangent_angles = (self.angles[:-1] - self.angles[1:]) / 2.

        # angles we try the car to get to
        # start with (current) angle and then try to get to next segment joint
        self.target_angles = np.hstack(self.angles[0], self.tangent_angles])

        # change in angle for each segment
        self.target_angle_diffs = self.target_angles[1:] - self.target_angles[:-1]


        # start with first segment
        self.segment = 0

        # reset odometry
        self.revolutions_anchor = self.last_revolutions

    def follow_trajectory(self):
        # only start when we have trajectory
        if self.resampled == None
            return
 
        self.speed = 0.150                      # in cm/s
        self.pub_vel.publish(self.speed/100.)   # in m/s

        # use odometry to determine if we reached the next segment
        if  self.distance_traveled >= self.lens[self.segment]:
            self.segment += 1
            self.distance_traveled -= self.lens[self.segment]

        # if we have no further data, repeat last
        if(self.segment < self.target_angle_diffs.shape[0]):
            self.segment = -1

        # determine how much time we need for this segment
        time = self.lens[self.segment] / self.speed

        # determine yaw rate
        target_angle_diff = self.target_angle_diffs[self.segment]

        # determine yaw rate
        yaw_rate = target_angle_diff / time

        self.pub_yaw.publish(yaw_rate);


    def callback_revolutions(self, msg):
        if(self.revolutions_anchor == None):
            self.revolutions_anchor = msg.data

        self.distance_traveled = (msg.data - self.revolutions_anchor) * self.revolutions_to_cm
        self.last_revolutions = msg.data

        # print self.distance_traveled

    def callback_rps(self, msg):
        self.speed = msg.data * self.revolutions_to_cm
        # if(abs(self.speed) < 0.01):
        #     self.revolutions_anchor = self.last_revolutions
        # print self.speed * 100, 'm/s', self.distance_traveled*100, 'm'
        print self.speed

    def spin(self):
        r = rospy.Rate(self.hz)
        while self.run:

            self.follow_traejctory()
            r.sleep()
        rospy.spin()


    def __init__(self):
        super(TrajectoryController, self).__init__()
        self.run = True
        self.pause = False
        signal.signal(signal.SIGINT, self.keyboard_interupt)  

        rospy.init_node('trajectory_controller')

        self.hz = 20

        self.revolutions_to_cm = (1./4765.7) * 100.
        self.revolutions_anchor = None
        self.distance_traveled = 0  # in [cm]
        self.speed = 0              # in [cm/s]

        self.last_revolutions = None

        self.resampled = None

        self.pub_yaw = rospy.Publisher('yaw_rate_cmd', Float32, tcp_nodelay=True)
        self.pub_vel = rospy.Publisher('velocity_cmd', Float32, tcp_nodelay=True)

        
        rospy.Subscriber('/ottocar_perception/lanes/mid', Float64MultiArray, self.callback_mid_lane)
        rospy.Subscriber('/sensor_motor_revolutions', Int32, self.callback_revolutions)
        # rospy.Subscriber('/sensor_motor_rps', Float32, self.callback_rps)
        self.spin()


if __name__ == '__main__':
    TrajectoryController()