#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Float32
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
        S = self.H * self.P * self.H.getT() + self.R        # sieht in wikipedia etwas anders aus..

        # K: Kalman-Gain
        K = self.P * self.H.getT() * S.getI()

        # x: Systemzustand
        self.x = self.x + K * w

        # P: Unsicherheit der Dynamik
        self.P = (self.I - K * self.H) * self.P

    def predict(self):
        # x: Systemzustand
        self.x = self.F * self.x + self.B * self.u

        # P: Unsicherheit der Dynamik
        self.P = self.F * self.P * self.F.getT() + self.Q   

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

        if biases==None:
            self.update(Z)
        else:
            self.update(Z-biases)
        self.predict()

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

class MotorFilter(Kalman):
    """docstring for ImuSensorsFilter"""
    def __init__(self, dt=1):
        super(MotorFilter, self).__init__(n_states = 2, n_sensors = 1)

        self.dt = dt    # dt in [sec]

        #states:
        #  revolutions:      0
        #  rps:              1

        #sensors:
        #  revolutions:      0
        #  rps:              1

        # H: Messmatrix
        self.H = np.matrix( '1 0;'    #revolutions
                            '0 1'     #rps
                            )
        # F: Dynamik
        self.F = np.matrix([[1,dt],    #revolutions = revolutions + rps*dt
                            [0,1]     #rps = rps
                            ])

        # Q: Unsicherheit der Dynamik 
        self.Q = np.matrix(np.identity(self.n_states)) * 0.1    
             
        # P: Unsicherheit ueber Systemzustand   
        self.P *= 0.1

        # R: Messunsicherheit
        self.R *= 1

    def update_dt(self,dt): # dt in [sec]
        self.F[0,1] = dt;

    def measure(self,revolutions,rps):
        Z = np.matrix([revolutions,rps]).getT()

        self.update(Z)
        self.predict()

class Subscriber(object):
    """docstring for Subscriber"""
    def __init__(self):
        super(Subscriber, self).__init__()
        rospy.init_node('kalman', anonymous=True)

        self.dt = 1./80. # topic rate ~ 80hz

        self.sensors = ImuSensorsFilter()
        self.sensor_biases = ImuSensorsBiasFilter()
        self.motor = MotorFilter(self.dt)    


        # Publishers
        self.pub_imu = rospy.Publisher('/imu/data_filtered', Imu)
        self.pub_imu_bias = rospy.Publisher('/imu/data_biases', Imu)
        self.pub_mag = rospy.Publisher('/imu/mag_filtered', Vector3Stamped)
        self.pub_rps = rospy.Publisher('/rps_filtered', Float32)

        # Subscribers
        rospy.Subscriber('/imu/data_raw', Imu, self.callback_imu)
        rospy.Subscriber('/imu/mag', Vector3Stamped, self.callback_mag)
        rospy.Subscriber('/sensor_distance', Int32, self.callback_revolutions)

        self.imu = self.mag = self.revolutions = None
        self.last_revolutions = self.last_time = None
        self.lock = threading.Lock()

        rospy.spin()

    def callback_revolutions(self, msg):
        self.revolutions = msg.data

        self.lock.acquire()
        self.measure()
        self.lock.release()

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
        if((self.mag==None)or(self.imu==None)or(self.revolutions==None)):
            return

        if(self.last_revolutions==None):
            self.last_revolutions=self.revolutions

        if(self.last_time==None):
            self.last_time=self.imu.header.stamp.to_sec()
        # else:
            # self.dt=

        # update bias if revolutions don't change, i.e. the car is standing still
        if(self.revolutions-self.last_revolutions == 0):
            self.sensor_biases.measure(self.imu,self.mag)

        # update sensors
        self.sensors.measure(self.imu,self.mag,self.sensor_biases.x)

        # update motor
        if(self.dt!=0):
            rps = (self.revolutions - self.last_revolutions) / self.dt # that is nasty, extended kalman filter needed...
        else:
            rps = self.motor.x[1]

        # print self.motor.x
        # self.motor.x[1] = rps

        self.motor.measure(self.revolutions,rps)


        header = self.mag.header

        # publish filtered data
        imu = Imu()
        imu.header = header
        imu.linear_acceleration.x = self.sensors.x[0]
        imu.linear_acceleration.y = self.sensors.x[1]
        imu.linear_acceleration.z = self.sensors.x[2]
        imu.angular_velocity.x = self.sensors.x[3]
        imu.angular_velocity.y = self.sensors.x[4]
        imu.angular_velocity.z = self.sensors.x[5]
        self.pub_imu.publish(imu)

        imu = Imu()
        imu.header = header
        imu.linear_acceleration.x = self.sensor_biases.x[0]
        imu.linear_acceleration.y = self.sensor_biases.x[1]
        imu.linear_acceleration.z = self.sensor_biases.x[2]
        imu.angular_velocity.x = self.sensor_biases.x[3]
        imu.angular_velocity.y = self.sensor_biases.x[4]
        imu.angular_velocity.z = self.sensor_biases.x[5]
        self.pub_imu_bias.publish(imu)

        mag = Vector3Stamped()
        mag.header = header
        mag.vector.x = self.sensors.x[6]
        mag.vector.y = self.sensors.x[7]
        mag.vector.z = self.sensors.x[8]
        self.pub_mag.publish(mag)

        self.pub_rps.publish(self.motor.x[1])

        self.last_revolutions=self.revolutions

        # remove old msgs
        self.imu = self.mag = self.revolutions = None


        


if __name__ == '__main__':
    subscriber = Subscriber()


