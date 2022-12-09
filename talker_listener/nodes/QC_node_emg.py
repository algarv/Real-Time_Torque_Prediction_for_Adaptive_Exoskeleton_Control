#!/usr/bin/env python
'''
QC_node_emg Node

This node subscribes to the torque measurements from the exoskeleton and the high-density EMG data from the intermediate streaming node and predicts torque_cmds
to send to the h3 in real time. 

Subscribers:
    Name: /h3/robot_states Type: h3_msgs/State         Published Rate: 100 Hz 
    
    Name: /hdEMG_stream    Type: talker_listener/hdemg Published Rate: 100 Hz 

Publishers:
    Name: /h3/right_ankle_effort_controller/command' Type: Float64
'''

import rospy
import random
import numpy as np
import pandas as pd
from std_msgs.msg import Float64, Float64MultiArray
from talker_listener.qc_predict import MUdecomposer
from talker_listener.msg import hdemg
from rospy.core import loginfo
from h3_msgs.msg import State
from scipy import signal
import matplotlib.pyplot as plt
import message_filters
from sklearn.gaussian_process import GaussianProcessRegressor
import joblib


path = rospy.get_param("/file_dir")

# Filter parameters #
nyquist = .5 * 100.0
emg_window = 100 #samples
torque_window = 50 #samples

# Configuration for EMG data #
muscles = [2, 3, 4]
n = len(muscles)
noisy_channels = [[],[],[]]

class QC_node:
    def __init__(self):

        r = rospy.Rate(2048)
        self.dt = 1.0 / 100.0

        self.torque_pub = rospy.Publisher('/h3/right_ankle_effort_controller/command', Float64, queue_size=10)
        self.sensor_sub = rospy.Subscriber('/h3/robot_states', State, self.sensor_callback)
        self.emg_sub = rospy.Subscriber('hdEMG_stream', hdemg, self.get_sample)

        # Timer to send torque commands at 100 Hz
        rospy.Timer(rospy.Duration(.01),self.send_torque_cmd)
        
        # Initialize arrays for data collection
        self.first_run = True
        self.batch_ready = False
        self.sample_count = 0
        self.emg_array = []
        self.raw_emg_array = []
        self.emg_win = []
        self.smooth_emg_win = []
        self.theta = 0

        self.torque_cmd = 0
        rospy.wait_for_message('/h3/robot_states', State,timeout=None)
        rospy.wait_for_message('hdEMG_stream', State,timeout=None)
        while not rospy.is_shutdown():
            if rospy.get_param('calibrated') == True:
                if self.first_run == True:
                    self.sample_count = 0
                    self.torque_max = rospy.get_param('Max_Torque', 35.0)
                    # self.gpr = joblib.load(path + "/src/talker_listener/emg_gpr")

                self.first_run = False

                self.torque_cmd = self.calc_torque_emg()

                r.sleep()

    def send_torque_cmd(self, event=None):
        ''' Callback for 100 Hz timer to publish torque command messages
        '''

        if rospy.get_param('calibrated') == True:
            rospy.loginfo("Torque_CMD: ")
            rospy.loginfo(self.torque_cmd)
            # if self.torque_cmd is not None:
            #     if self.torque_cmd > torque_max: 
            #         self.torque_cmd = torque_max
            #         rospy.WARN("TORQUE_CMD Exceeded Max")

            self.torque_pub.publish(self.torque_cmd)

    def get_sample(self,hdEMG):
        ''' Callback for the /hdEMG topic. 
        Organize the raw data into muscle groups, calculate the RMS of each channel over time and average each muscle group. Updates the class variable sample_array with the RMS emg values for
        each muscle and current joint angle measurement to create a 4 element list for torque estimation.   
        '''

        reading = hdEMG.data.data
        num_groups = len(reading) // 64
        self.raw_emg_array.append(reading)

        samples = []
        for j in range(num_groups):
            muscle = list(reading[64*j : 64*j + 64])
            if j in muscles:
                samples.append([m**2 for m in muscle])

        if self.sample_count < emg_window:
            self.emg_win.append(samples)
        else:
            self.emg_win.pop(0)
            self.emg_win.append(samples)

        smoothed_reading =  np.sqrt(np.mean(self.emg_win, axis=0))

        sample = []
        for j in range(n):
            sample.append(np.mean(smoothed_reading[j]))
        
        self.emg_array.append(sample)

        self.sample_array = sample + [self.theta]

        self.sample_count += 1        

    def sensor_callback(self,sensor_reading):
        ''' Callback for /h3/robot_states. Reads sensor messages from the h3 and saves them in class variables.
        '''

        self.theta = sensor_reading.joint_position[2]
        self.theta_dot = sensor_reading.joint_velocity[2]
        self.net_torque = sensor_reading.joint_torque_sensor[2]
        self.theta_next = self.theta + self.theta_dot * self.dt

    def calc_torque_emg(self):
        ''' Predicts torque using the model fit in the calibration node and the sample array created in the hdEMG callback.

        Returns:
            torque_cmd (float): Torque command
        '''

        coef = rospy.get_param('emg_coef')

        if len(self.emg_array) > 0:

            torque_cmd = self.f(pd.DataFrame(self.sample_array), coef)

            return -1*torque_cmd[0]
        
    def f(self, X, betas):
        '''The model to predict torque from RMS EMG

        Args:
            X (Data Frame): Data frame containing a column for each muscle and one column for joint angle        
        Returns:
            f (float[]): Array of emg predictions
        '''

        ones = pd.DataFrame({'ones': np.ones(X.iloc[1,:].size) }).T
        zeros = pd.DataFrame({'output': np.zeros(X.iloc[1,:].size) }).T
        
        RMS_TA = X.iloc[0,:]
        RMS_GM = X.iloc[1,:]
        RMS_SOL = X.iloc[2,:]
        theta = X.iloc[3,:]

        f = (betas[0] * (RMS_TA - betas[1])).to_numpy() + (betas[2] * (RMS_TA*theta - betas[3])).to_numpy() + (betas[4] * (RMS_GM - betas[5])).to_numpy() + (betas[6] * (RMS_GM*theta - betas[7])).to_numpy() + (betas[8] * (RMS_SOL - betas[9])).to_numpy() + (betas[10] * (RMS_SOL*theta - betas[11])).to_numpy() + (betas[12]*(theta-betas[13])).to_numpy() + (betas[14] * ones).to_numpy() 

        return f[0]

if __name__ == '__main__':
    try:
        rospy.init_node('QC_node_emg', log_level=rospy.DEBUG)
        node = QC_node()
    except rospy.ROSInterruptException:
        pass
