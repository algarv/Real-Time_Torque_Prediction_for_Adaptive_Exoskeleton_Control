#!/usr/bin/env python3

from cv2 import log
import rospy
import numpy as np
import scipy as sp
from scipy import signal
from std_msgs.msg import Float64, Float64MultiArray
from sklearn import linear_model as lm
import matplotlib.pyplot as plt
from rospy.core import logdebug
from h3_msgs.msg import State

trial_length = 26 #seconds
rest_time = 15 #seconds
torque_window = 10**7 #samples (100 Hz * 100 ms Window)

class calibrate:
    def __init__(self):

        self.fig, self.axs = plt.subplots()
        self.start_time = rospy.Time.now().to_sec()

        self.r = rospy.Rate(100)
        torque_sub = rospy.Subscriber('/h3/robot_states', State, self.torque_calib)
        emg_sub = rospy.Subscriber('hdEMG_stream', Float64MultiArray, self.emg_calib)
        self.pos_pub = rospy.Publisher('/h3/right_ankle_position_controller/command',Float64, queue_size=10)
        self.max_torque = 0
        self.torque_array = []
        self.smoothed_torque_array = []
        self.PF0_array = []
        self.PF10_array = []
        self.DF0_array = []
        self.DF10_array = []
        self.emg_array = []
        self.time = []

        logdebug("Starting PFO")
        self.PF0()
    
    def max(self):

        start_index = len(self.torque_array)
        start = rospy.Time.now()
        duration = rospy.Duration.from_sec(5) 
        while (rospy.Time.now() < start + duration):
            self.r.sleep()

        MVC = np.max(self.torque_array[start_index:]) #moving window average filter?
        return MVC

    def get_max(self):
        logdebug("Go!")
        max1 = self.max()
        logdebug("MVC 1: ")
        logdebug(max1)

        logdebug("REST")
        rospy.sleep(5)

        logdebug("Go!")
        max2 = self.max()
        logdebug("MVC 2: ")
        logdebug(max2)

        return float((max1 + max2)/2)

    def make_traj_trap(self, min, max, len):
        desired_traj = [0, 0, 0]
        step =  .20 * (max-min) / 5
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

        desired_traj.append(0)
        desired_traj.append(0)
        desired_traj.append(0)

        return desired_traj

    def PF0(self):
        logdebug("Moving to 0")
        self.pos_pub.publish(float(0.0))
        rospy.sleep(5)

        logdebug("Apply Max PF Torque")
        
        self.max_PF0 = self.get_max()
        rospy.set_param('max_PF0', self.max_PF0)

        desired_traj = self.make_traj_trap(0,self.max_PF0,25)

        self.axs.plot(desired_traj, color='blue')
        self.axs.set_xlim(0, 25)
        self.axs.set_ylim(0, 1.25*self.max_PF0)

        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.emg_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)
        i = 0
        while (rospy.Time.now() < (start + duration)):
            i += 1
            self.axs.plot(self.time, self.smoothed_torque_array[0:len(self.time)], color='red')
            plt.pause(.01)
            self.r.sleep()

        self.PF0_torque_array = self.smoothed_torque_array
        self.PF0_emg_array = self.emg_array
        
        logdebug("REST")
        rospy.sleep(rest_time)
        plt.close()
        logdebug("Starting PF20")
        self.PF20()

    def PF20(self):
        self.fig, self.axs = plt.subplots()

        logdebug("Moving to 10 degrees")
        self.pos_pub.publish(0.17)
        rospy.sleep(5)

        self.max_PF20 = self.get_max()
        rospy.set_param('max_PF20', self.max_PF20)

        desired_traj = self.make_traj_trap(0,self.max_PF20,25)
        self.axs.plot(desired_traj, color='blue')
        self.axs.set_xlim(0, 26)
        self.axs.set_ylim(0, 1.25*self.max_PF20)

        #self.calib_index = len(self.torque_array)
        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.emg_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)
        while (rospy.Time.now() < start + duration):
            self.axs.plot(self.time, self.smoothed_torque_array[0:len(self.time)], color='red')
            plt.pause(.01)
            self.r.sleep()

        self.PF20_torque_array = self.smoothed_torque_array
        self.PF20_emg_array = self.emg_array

        logdebug("REST")
        rospy.sleep(rest_time)
        plt.close()
        logdebug("Starting PFn20")
        self.PFn20()

    def PFn20(self):
        self.fig, self.axs = plt.subplots()

        logdebug("Moving to -10 degrees")
        self.pos_pub.publish(-0.17)
        rospy.sleep(5)

        self.max_PFn20 = self.get_max()
        rospy.set_param('max_PF20', self.max_PFn20)

        desired_traj = self.make_traj_trap(0,self.max_PFn20,25)
        self.axs.plot(desired_traj, color='blue')
        self.axs.set_xlim(0, 26)
        self.axs.set_ylim(0, 1.25*self.max_PFn20)

        #self.calib_index = len(self.torque_array)
        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.emg_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)
        while (rospy.Time.now() < start + duration):
            self.axs.plot(self.time, self.smoothed_torque_array[0:len(self.time)], color='red')
            plt.pause(.01)
            self.r.sleep()

        self.PFn20_torque_array = self.smoothed_torque_array
        self.PFn20_emg_array = self.emg_array

        logdebug("REST")
        rospy.sleep(rest_time)
        plt.close()
        logdebug("Starting DF0")
        self.DF0()

    def DF0(self):
        self.fig, self.axs = plt.subplots()

        logdebug("Moving to 0 degrees")
        self.pos_pub.publish(0.0)
        rospy.sleep(5)

        self.max_DF0 = self.get_max()
        rospy.set_param('max_DF0', self.max_DF0)

        desired_traj = self.make_traj_trap(0,self.max_PF20,25)
        self.axs.plot(desired_traj, color='blue')
        self.axs.set_xlim(0, 26)
        self.axs.set_ylim(0, 1.25*self.max_PF0)

        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.emg_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)
        while (rospy.Time.now() < start + duration):
            self.axs.plot(self.time, self.smoothed_torque_array[0:len(self.time)], color='red')
            plt.pause(.01)
            self.r.sleep()

        self.DF0_torque_array = self.smoothed_torque_array
        self.DF0_emg_array = self.emg_array

        logdebug("REST")
        rospy.sleep(rest_time)
        plt.close()
        logdebug("Starting DF20")
        self.DF20()

    def DF20(self):
        self.fig, self.axs = plt.subplots()

        logdebug("Moving to 20 degrees")
        self.pos_pub.publish(-0.17)
        rospy.sleep(5)

        self.max_DF20 = self.get_max()
        rospy.set_param('max_DF20', self.max_DF20)

        desired_traj = self.make_traj_trap(0,self.max_DF20,25)
        self.axs.plot(desired_traj, color='blue')
        self.axs.set_xlim(0, 26)
        self.axs.set_ylim(0, 1.25*self.max_PF0)

        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.emg_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)
        while (rospy.Time.now() < start + duration):
            self.axs.plot(self.time, self.smoothed_torque_array[0:len(self.time)], color='red')
            plt.pause(.01)
            self.r.sleep()

        self.DF20_torque_array = self.smoothed_torque_array
        self.DF20_emg_array = self.emg_array

        logdebug("Starting Calibration")
        self.calibration()

    def calibration(self):
        y = np.concatenate((self.PF20_torque_array, self.PF0_torque_array, self.PFn20_torque_array, self.DF0_torque_array, self.DF20_torque_array)).reshape(-1,1)
        
        x1 = signal.resample(self.PF20_emg_array, len(self.PF20_torque_array))
        x2 = signal.resample(self.PF0_emg_array, len(self.PF0_torque_array))
        x3 = signal.resample(self.PFn20_emg_array, len(self.PFn20_torque_array))
        x4 = signal.resample(self.DF0_emg_array, len(self.DF0_torque_array))
        x5 = signal.resample(self.DF20_emg_array, len(self.DF20_torque_array))

        self.DF20_torque_array = signal.resample(self.DF20_torque_array, len(x5))

        x = np.concatenate((x1, x2, x3, x4, x5)).reshape(-1,1)
        
        model = lm.LinearRegression()
        res=model.fit(x,y)
        
        intercept = float(res.intercept_[0])
        slope = float(res.coef_[0][0])

        logdebug(intercept)
        logdebug(slope)

        rospy.set_param('intercept',intercept)
        rospy.set_param('slope',slope)
        rospy.set_param('calibrated', True)

    def torque_calib(self,sensor_reading):
        torque = sensor_reading.joint_torque_sensor[2]
        # if len(self.torque_array) > 0:
        #     torque -= self.torque_array[0]
        
        self.torque_array.append(abs(torque))
        if len(self.torque_array) > 0:
            if (len(self.torque_array) <= torque_window):
                avg = np.sum(self.torque_array[0:]) / len(self.torque_array)
                self.smoothed_torque_array.append(avg)
            else:
                avg = np.sum(self.torque_array[-1*torque_window:-1]) / torque_window
                self.smoothed_torque_array.append(avg)
        
            self.time.append(rospy.Time.now().to_sec() - self.start_time)

            if self.smoothed_torque_array[-1] > self.max_torque:
                rospy.set_param('Max_Torque', torque)    
                self.max_torque = torque

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