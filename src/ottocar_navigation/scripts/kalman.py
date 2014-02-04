#!/usr/bin/env python
import rospy
import numpy as np
import numdifftools as nd   # sudo pip install numdifftools

from time import time

from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

import functools
import sys
import signal

import threading

class Kalman(object):
    """docstring for Kalman"""
    # http://www.cbcity.de/das-kalman-filter-einfach-erklaert-teil-2
    # http://de.wikipedia.org/wiki/Kalman-Filter
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

    def reset(self):
        self.x *= 0

    def update(self, Z):
        '''Z: new sensor values as numpy matrix'''

        # print 'Z.shape', Z.shape
        # print 'self.H.shape', self.H.shape
        # print 'self.x.shape', self.x.shape

        # w: Innovation
        w = Z - self.H * self.x
        #http://services.eng.uts.edu.au/~sdhuang/1D%20Kalman%20Filter_Shoudong.pdf 
        # gibt noch einen zusaetzlichen 'zero-mean Gaussian observation noise' v an, der drauf addiert wird (in gleichung (2))
        # in Gleichung (7) wird es jedoch nicht mit angegeben
        #http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf
        # Gleichung(7)
        # beim EKF wird es zu w=Z-h(x)
        # h: observation function

        # S: Residualkovarianz (http://de.wikipedia.org/wiki/Kalman-Filter#Korrektur)
        S = self.H * self.P * self.H.getT() + self.R        # sieht in wikipedia etwas anders aus..
        #http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf
        # Gleichung(9)
        # beim EKF wird es zu S=J_h*P*J_h^T + R
        # J_h:Jacobian of function h evaluated at next x, i.e. x after this update -> calculate x before S.

        # K: Kalman-Gain
        K = self.P * self.H.getT() * S.getI()
        #http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf
        # Gleichung(10)
        # beim EKF wird es zu K=P*J_h^T * S^(-1)
        # J_h:Jacobian of function h evaluated at next x, i.e. x after this update -> calculate x before S.

        # x: Systemzustand
        self.x = self.x + K * w


        # P: Unsicherheit der Dynamik
        # self.P = (self.I - K * self.H) * self.P
        self.P = self.P - K * S * K.getT()
        #http://services.eng.uts.edu.au/~sdhuang/1D%20Kalman%20Filter_Shoudong.pdf 
        # ist in Gleichung (8) anders angegeben, vlt ist das aequivalent??

    def predict(self):
        # x: Systemzustand
        self.x = self.F * self.x + self.B * self.u  
        #http://services.eng.uts.edu.au/~sdhuang/1D%20Kalman%20Filter_Shoudong.pdf 
        # gibt noch einen zusaetzlichen 'zero-mean Gaussian process noise' w an, der drauf addiert wird (in Gleichung (1))
        # in Gleichung (5) wird es jedoch nicht mit angegeben, vlt ist w in G und u integriert?
        #http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf
        # Gleichung(5)
        # beim EKF wird es zu x=f(x,u)

        # P: Unsicherheit der Dynamik
        self.P = self.F * self.P * self.F.getT() + self.Q   
        #http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf
        # Gleichung(6)
        # beim EKF wird es zu P=J_f*P*J_f^T+Q
        # J_f:Jacobian of function f with respect to x evaluated at current x.

class ExtendedKalman(object):
    """docstring for Kalman"""
    # http://www.cbcity.de/das-kalman-filter-einfach-erklaert-teil-2
    # http://de.wikipedia.org/wiki/Kalman-Filter
    # http://services.eng.uts.edu.au/~sdhuang/1D%20Kalman%20Filter_Shoudong.pdf
    # http://services.eng.uts.edu.au/~sdhuang/Kalman%20Filter_Shoudong.pdf
    # http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf
    def __init__(self, n_states, n_sensors):
        super(ExtendedKalman, self).__init__()
        self.n_states = n_states
        self.n_sensors = n_sensors

        # x: Systemzustand
        self.x = np.matrix(np.zeros(shape=(n_states,1)))

        # P: Unsicherheit ueber Systemzustand
        self.P = np.matrix(np.identity(n_states)) 

        # F: Dynamik
        # x,u,and return type are np.matrix
        self.f = lambda x,u: np.matrix(np.identity(n_states)) * x

        # Q: Dynamik Unsicherheit
        # self.Q = np.matrix(np.zeros(shape=(n_states,n_states)))
        self.Q = np.matrix(np.identity(n_states)) 

        # u: externe Beeinflussung des Systems
        self.u = np.matrix(np.zeros(shape=(n_states,1)))

        # B: Dynamik der externen Einfluesse
        self.B = np.matrix(np.identity(n_states))

        # h: Messfunktion
        # x,and return type are np.matrix
        # function h can be used to compute the predicted measurement from the predicted state
        #  (http://www.lr.tudelft.nl/fileadmin/Faculteit/LR/Organisatie/Afdelingen_en_Leerstoelen/Afdeling_AEWE/Applied_Sustainable_Science_Engineering_and_Technology/Education/AE4-T40_Kites,_Smart_kites,_Control_and_Energy_Production/doc/Lecture5.ppt)
        self.h = lambda x: np.matrix(np.zeros(shape=(n_sensors, 1)))

        # R: Messunsicherheit
        self.R = np.matrix(np.identity(n_sensors))

        # I: Einheitsmatrix
        self.I = np.matrix(np.identity(n_states))

        self.first = True

    # jacobi, differentiation:
    # https://code.google.com/p/numdifftools/   (numerical) easiest to use, so just use this
    # http://sympy.org/en/index.html            (symbolic)
    # http://openopt.org/FuncDesigner           (automatic, better than numerical)

    def reset(self):
        self.x *= 0

    def update(self, Z):
        '''Z: new sensor values as numpy matrix'''

        # print Z

        # w: Innovation (http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf Eq. 7)
        w = Z - np.matrix(self.h(np.array(self.x)))
        # print w
# 
        # J_h:Jacobian of function h evaluated at current x
        # conversions between np.array and np.matrix are necessary because nd.Jacobian needs np.array, but we use np.matrix everywhere
        J_h = nd.Jacobian(self.h)
        J_h = np.matrix(J_h(np.array(self.x)))

        # S: Residualkovarianz (http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf Eq. 9)
        S = J_h  * self.P * J_h.getT() + self.R

        # K: Kalman-Gain (http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf Eq. 10)
        K = self.P * J_h.getT() * S.getI()


        # x: Systemzustand
        self.x = self.x + K * w

        # P: Unsicherheit der Dynamik (http://services.eng.uts.edu.au/~sdhuang/1D%20Kalman%20Filter_Shoudong.pdf Eq. 8)
        self.P = self.P - K * S * K.getT()

    def predict(self):
        # x: Systemzustand (http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf Eq. 5)
        # print self.x
        self.x = np.matrix(self.f(np.array(self.x), np.array(self.u)))
        # print self.x

        # J_f:Jacobian of function f with respect to x evaluated at current x.
        # conversions between np.array and np.matrix are necessary because nd.Jacobian only works with np.array, but we use np.matrix everywhere
        J_f = nd.Jacobian(lambda x: self.f(x, self.u))
        J_f = np.matrix(J_f(np.array(self.x)))


        # print J_f
        # print self.Q
        # sys.exit()

        # P: Unsicherheit der Dynamik (http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf Eq. 6)
        self.P = J_f * self.P * J_f.getT() + self.Q

class ImuSensorsFilter(Kalman):
    """docstring for ImuSensorsFilter"""
    def __init__(self):
        super(ImuSensorsFilter, self).__init__(n_states = 9, n_sensors = 9)
        #states:
        #  accel: x,y,z     0:3
        #  gyro:  x,y,z     3:6
        #  mag:   x,y,z     6:9        

        #sensors:
        #  accel: x,y,z     0:3
        #  gyro:  x,y,z     3:6
        #  mag:   x,y,z     6:9        

        self.states = self.sensors = {'accel.x': 0, 'accel.y': 1, 'accel.z': 2,
                                      'gyro.x':  3, 'gyro.y':  4, 'gyro.z':  5,
                                      'mag.x':   6, 'mag.y':   7, 'mag.z':   8}

        # H: Messmatrix
        self.H = np.matrix( '1 0 0 0 0 0 0 0 0;'    #accel.x
                            '0 1 0 0 0 0 0 0 0;'    #accel.y
                            '0 0 1 0 0 0 0 0 0;'    #accel.z
                            '0 0 0 1 0 0 0 0 0;'    #gyro.x
                            '0 0 0 0 1 0 0 0 0;'    #gyro.y
                            '0 0 0 0 0 1 0 0 0;'    #gyro.z
                            '0 0 0 0 0 0 1 0 0;'    #mag.x
                            '0 0 0 0 0 0 0 1 0;'    #mag.y
                            '0 0 0 0 0 0 0 0 1 '    #mag.z
                            )
        # F: Dynamik
        self.F = np.matrix([[1,0,0,0,0,0,0,0,0],    #accel.x = accel.x
                            [0,1,0,0,0,0,0,0,0],    #accel.y = accel.y
                            [0,0,1,0,0,0,0,0,0],    #accel.z = accel.z
                            [0,0,0,1,0,0,0,0,0],    #gyro.x = gyro.x
                            [0,0,0,0,1,0,0,0,0],    #gyro.y = gyro.y
                            [0,0,0,0,0,1,0,0,0],    #gyro.z = gyro.z
                            [0,0,0,0,0,0,1,0,0],    #mag.x = mag.x
                            [0,0,0,0,0,0,0,1,0],    #mag.y = mag.y
                            [0,0,0,0,0,0,0,0,1]     #mag.z = mag.z
                            ])

        # Q: Unsicherheit der Dynamik 
        self.Q = np.matrix(np.identity(self.n_states)) * 0.1    
             
        # P: Unsicherheit ueber Systemzustand   
        self.P *= 0.1

        # R: Messunsicherheit
        self.R *= 1


    def measure(self,imu,mag,biases=None):
        Z = np.matrix([imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z,
                       imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z,
                       mag.vector.x,mag.vector.y,mag.vector.z]).getT()

        self.predict()
        if biases==None:
            self.update(Z)
        else:
            self.update(Z-biases)

class ImuSensorsBiasFilter(ImuSensorsFilter):
    """docstring for ImuSensorsBiasFilter"""
    def __init__(self):
        super(ImuSensorsBiasFilter, self).__init__()

        # Aenderungen gegenueber ImuSensorsFilter:

        # Q: Unsicherheit der Dynamik 
        self.Q *= 1
             
        # P: Unsicherheit ueber Systemzustand   
        self.P *= 1

        # R: Messunsicherheit
        self.R *= 1
 
class MotionModelCV(ExtendedKalman):
    """Constant Velocity linear motion model"""
    #http://www.isif.org/fusion/proceedings/fusion08CD/papers/1569107835.pdf
    def __init__(self, dt):
        super(MotionModelCV, self).__init__(n_states = 3, n_sensors = 1)
        #states:
        #  velocity      0
        #  acceleration  1
        #  distance      2

        #sensors:
        #  acceleration  0

        self.states = {'velocity': 0, 'acceleration': 1, 'distance': 2}
        self.sensors = {'acceleration': 0}

        # self.x[1,0] = 10

        self.dt = dt
        # F: Dynamik
        self.f_dt = lambda x,u,dt: np.array([
            [x[0,0] + dt*x[1,0]],                   # velocity     = velocity + dt * acceleration
            [0],                                    # acceleration = 0
            [x[2,0] + dt*x[0,0] + dt*dt*x[1,0]],    # position     = position + dt * velocity + dt * dt * acceleration
            ])                   
        self.f = functools.partial(self.f_dt, dt=self.dt)

        # h: Messfunktion
        # function h can be used to compute the predicted measurement from the predicted state
        self.h = lambda x: np.array([x[1,0]])

    def update_dt(self,dt):
        self.dt = dt
        self.f = functools.partial(self.f_dt, dt=self.dt)

    def measure(self,acceleration):
        # print acceleration
        # Z = np.matrix([[acceleration]])
        Z = np.matrix([acceleration]).getT()

        self.predict()

        self.x[1,0] = acceleration
        
        self.update(Z)

class Subscriber(object):
    """docstring for Subscriber"""
    def __init__(self):
        super(Subscriber, self).__init__()
        rospy.init_node('kalman', anonymous=True)


        self.run = True
        self.pause = False
        self.rate = 40
        signal.signal(signal.SIGINT, self.keyboard_interupt)
        
        self.dt = 1./min(self.rate,40.) # topic rate ~ 40hz


        self.sensors = ImuSensorsFilter()
        self.sensor_biases = ImuSensorsBiasFilter()

        self.motion_cv = MotionModelCV(dt=self.dt)

        # Publishers
        self.pub_imu = rospy.Publisher('/imu/data_filtered', Imu)
        self.pub_imu_bias = rospy.Publisher('/imu/data_biases', Imu)
        self.pub_mag = rospy.Publisher('/imu/mag_filtered', Vector3Stamped)
        self.pub_rps = rospy.Publisher('/rps', Float32)

        self.pub_cv_vel = rospy.Publisher('/motion/cv/velocity', Float32)
        self.pub_cv_acc = rospy.Publisher('/motion/cv/acceleration', Float32)

        # Subscribers
        rospy.Subscriber('/imu/data_raw', Imu, self.callback_imu)
        rospy.Subscriber('/imu/mag', Vector3Stamped, self.callback_mag)
        rospy.Subscriber('/sensor_distance', Int32, self.callback_revolutions)

        self.rps_gain = 0.25
        self.dt_gain = 0.125

        self.imu = self.mag = self.revolutions = None
        self.rps = 0
        self.last_revolutions = self.last_rps_time = self.last_time = None


        self.counter = 0

        self.spin()

    def reset(self):
        self.dt = 1./min(self.rate,40.) # topic rate ~ 40hz
        
        self.sensors.reset()
        self.sensor_biases.reset()
        self.motion_cv.reset()

        self.imu = self.mag = self.revolutions = None
        self.rps = 0
        self.last_revolutions = self.last_rps_time = self.last_time = None

        print "reset"

    def callback_revolutions(self, msg):
        self.revolutions = msg.data

        # update rps 
        if(self.last_revolutions==None):
            self.last_revolutions=self.revolutions
        if(self.last_rps_time==None):
            self.last_rps_time=time()

        dt = time()-self.last_rps_time
        if(dt!=0):
            rps_now = (self.revolutions - self.last_revolutions) / dt
            self.rps = self.rps_gain * rps_now + (1-self.rps_gain) * self.rps

        self.last_revolutions=self.revolutions                    
        self.last_rps_time=time()

    def callback_imu(self, msg):
        self.imu = msg

    def callback_mag(self, msg):
        self.mag = msg

        if(msg.header.seq == 9379):
            self.reset()



    def spin(self):
        # print "Setting up rate ", self.rate, "hz"
        r = rospy.Rate(self.rate)
        while(self.run):
            self.measure()
            # self.pub_mag.publish(Vector3Stamped())
            r.sleep()


    def measure(self):
        # only proceed if we have all msgs
        if((self.mag==None)or(self.imu==None)or(self.rps==None)):
            return

        # filter dt
        if(self.last_time == None):
            self.last_time = time()
        dt_now = time()-self.last_time
        self.dt = self.dt_gain * dt_now + (1-self.dt_gain) * self.dt

        self.counter += 1
        if(self.counter==10):
            print 1/self.dt
            self.counter=0

        # update bias if car stands still (rps < 1)
        if(self.rps < 1):
            self.sensor_biases.measure(self.imu,self.mag)

        # update sensors
        self.sensors.measure(self.imu,self.mag,self.sensor_biases.x)

        header = self.mag.header

        # update motion model

        self.motion_cv.update_dt(self.dt)
        self.motion_cv.measure(self.sensors.x[self.sensors.states['accel.x'],0])

        ######### publish filtered data

        # publish filtered imu
        imu = Imu()
        imu.header = header
        imu.linear_acceleration.x = self.sensors.x[0]
        imu.linear_acceleration.y = self.sensors.x[1]
        imu.linear_acceleration.z = self.sensors.x[2]
        imu.angular_velocity.x = self.sensors.x[3]
        imu.angular_velocity.y = self.sensors.x[4]
        imu.angular_velocity.z = self.sensors.x[5]
        self.pub_imu.publish(imu)

        # publish imu bias
        imu = Imu()
        imu.header = header
        imu.linear_acceleration.x = self.sensor_biases.x[0]
        imu.linear_acceleration.y = self.sensor_biases.x[1]
        imu.linear_acceleration.z = self.sensor_biases.x[2]
        imu.angular_velocity.x = self.sensor_biases.x[3]
        imu.angular_velocity.y = self.sensor_biases.x[4]
        imu.angular_velocity.z = self.sensor_biases.x[5]
        self.pub_imu_bias.publish(imu)

        # publish filtered mag
        mag = Vector3Stamped()
        mag.header = header
        mag.vector.x = self.sensors.x[6]
        mag.vector.y = self.sensors.x[7]
        mag.vector.z = self.sensors.x[8]
        self.pub_mag.publish(mag)

        # publish rps
        self.pub_rps.publish(self.rps)

        # publish data from motion model 
        self.pub_cv_vel.publish(self.motion_cv.x[self.motion_cv.states['velocity'],0])
        self.pub_cv_acc.publish(self.motion_cv.x[self.motion_cv.states['acceleration'],0])

        # remove old msgs
        self.imu = self.mag = self.revolutions = None

        self.last_time = time()


    def keyboard_interupt(self, signum, frame):
        self.run = False
        print " closing...\n"


if __name__ == '__main__':
    subscriber = Subscriber()


