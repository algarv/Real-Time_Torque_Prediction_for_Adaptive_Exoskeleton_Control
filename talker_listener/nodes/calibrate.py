#!/usr/bin/env python

from re import S
from turtle import st

from cv2 import log
import rospy 
import numpy as np
import scipy as sp
from scipy import signal
from std_msgs.msg import Float64, Float64MultiArray
from sklearn import linear_model as lm
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rospy.core import logdebug
from h3_msgs.msg import State

max_torque = 0
trial_length = 20 #seconds
rest_time = 30 #seconds

class calibrate:
    def __init__(self):

        self.fig, self.axs = plt.subplots()
        self.axs.set_xlim(0,trial_length)
        self.axs.set_ylim(-10,10)
        self.start_time = rospy.Time.now().to_sec()

        self.r = rospy.Rate(100)
        torque_sub = rospy.Subscriber('/h3/robot_states', State, self.torque_calib)
        emg_sub = rospy.Subscriber('hdEMG_stream', Float64MultiArray, self.emg_calib)
        self.pos_pub = rospy.Publisher('joint_position',Float64, queue_size=10)

        self.torque_array = []
        self.emg_array = []
        self.time = []

        self.PF0()
    
    def max(self):
        logdebug("Starting torque: ")
        logdebug(self.torque_array)
        start_index = len(self.torque_array)
        start = rospy.Time.now()
        duration = rospy.Duration.from_sec(5) 
        while (rospy.Time.now() < start + duration):
            # logdebug(self.time)
            logdebug(self.torque_array)
            # logdebug(self.emg_array)
            self.r.sleep()

        MVC = np.max(self.torque_array[start_index:]) #moving window average filter?
        return MVC

    def PF0(self):
        self.pos_pub.publish(0.0)
        max1 = self.max()
        max2 = self.max() 

        self.max_PF0 = float((max1 + max2)/2)
        rospy.set_param('max_PF0', self.max_PF0)

        logdebug("STARTING PF0")
        desired_traj = [0, 0, 0]
        step = .20 * self.max_PF0 / 5
        prev = 0
        for i in range(5):
            desired_traj.append(prev)
            prev += step
        for i in range(10):
            desired_traj.append(.20 * self.max_PF0)
        prev = desired_traj[-1]
        for i in range(5):
            desired_traj.append(prev)
            prev -= step
        #desired_traj = np.concatenate((np.linspace(0,int(.20 * self.max_PF0,5)),5*int(np.ones(.20 * self.max_PF0)),np.linspace(int(.20 * self.max_PF0,0,6))))
        desired_traj.append(0)
        desired_traj.append(0)
        desired_traj.append(0)
        self.axs.plot(desired_traj, color='blue')

        start_index = len(self.torque_array)
        start = rospy.Time.now()
        duration = rospy.Duration.from_sec(trial_length) 
        while (rospy.Time.now() < start + duration):
            # logdebug(self.time)
            logdebug(self.torque_array)
            # logdebug(self.emg_array)
            self.axs.plot(self.time[start_index:], self.torque_array[start_index:], color='red')
            plt.pause(.01)
            self.r.sleep()
        
        self.calibration()
        # self.PF20()

    def PF20(self):
        self.pos_pub.publish(0.34) 

        plt.cla()
        desired_traj = np.concatenate((np.linspace(0,5,5),5*np.ones(5),np.linspace(5,0,6)))
        self.axs.plot(desired_traj, color='blue')

        self.calib_index = len(self.torque_array)
        start_index = len(self.torque_array)
        start = rospy.Time.now()
        duration = rospy.Duration.from_sec(trial_length) 
        while (rospy.Time.now() < start + duration):
            # logdebug(time)
            # logdebug(torque_array)
            # logdebug(emg_array)
            self.axs.plot(self.time[start_index:], self.torque_array[start_index:], color='red')
            plt.pause(.01)
            self.r.sleep()

        # self.DF0()

    def DF0(self):
        self.pos_pub.publish(0.0)

        plt.clf()
        desired_traj = np.concatenate((np.linspace(0,5,5),5*np.ones(5),np.linspace(5,0,6)))
        self.axs.plot(desired_traj, color='blue')

        start = rospy.Time.now()
        duration = rospy.Duration.from_sec(trial_length) 
        while (rospy.Time.now() < start + duration):
            # logdebug(time)
            # logdebug(torque_array)
            # logdebug(emg_array)
            self.axs.plot(self.time, self.torque_array, color='red')
            plt.pause(.01)
            self.r.sleep()

        self.DF20()

    def DF20(self):
        self.pos_pub.publish(0.34)

        plt.clf()
        desired_traj = np.concatenate((np.linspace(0,5,5),5*np.ones(5),np.linspace(5,0,6)))
        self.axs.plot(desired_traj, color='blue')

        start = rospy.Time.now()
        duration = rospy.Duration.from_sec(trial_length) 
        while (rospy.Time.now() < start + duration):
            # logdebug(time)
            # logdebug(torque_array)
            # logdebug(emg_array)
            self.axs.plot(self.time, self.torque_array, color='red')
            plt.pause(.01)
            self.r.sleep()

        self.calibration()

    def calibration(self):
        y = np.array(self.torque_array[self.calib_index:]).reshape(-1,1)
        x = signal.resample(self.emg_array[self.calib_index:], len(y))
        x = np.array(x).reshape(-1,1)
        model = lm.LinearRegression()
        res=model.fit(x,y)
        intercept = res.intercept_[0]
        slope = res.coef_[0][0]
        intercept = float(intercept)
        slope = float(slope)
        logdebug(intercept)
        logdebug(slope)
        rospy.set_param('intercept',intercept)
        rospy.set_param('slope',slope)
        rospy.set_param('calibrated', True)

    def torque_calib(self,sensor_reading):
        torque = sensor_reading.joint_torque_sensor[2]
        self.torque_array.append(abs(torque))
        self.time.append(rospy.Time.now().to_sec() - self.start_time)
        if torque > max_torque:
            rospy.set_param('Max_Torque', torque)    

    def emg_calib(self,hdEMG):
        sample = np.mean(hdEMG.data)
        self.emg_array.append(sample) #pick specific value?


if __name__ == '__main__':
    try:
        rospy.init_node('calibration', log_level=rospy.DEBUG)
        calibration = calibrate()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass