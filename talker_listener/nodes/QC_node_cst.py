#!/usr/bin/env python
'''
QC_node_cst Node

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

# Neural Net Set-Up #
path = rospy.get_param("/file_dir")
#model_file = path + "\\src\\talker_listener\\best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model_file = path + "/src/talker_listener/" + "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model = MUdecomposer(model_file)

# Filter parameters #
nyquist = .5 * 512

torque_window = 50 #samples
cst_window = 40
prediction_step_size = 20

# Configuration for EMG data #
muscles = [2, 3, 4] 
n = len(muscles)
noisy_channels = [[],[],[]]


class QC_node:
    def __init__(self):
        
        r = rospy.Rate(2048)
        self.dt = 1.0 / 512.0
        
        self.torque_pub = rospy.Publisher('/h3/right_ankle_effort_controller/command', Float64, queue_size=10)
        self.sensor_sub = rospy.Subscriber('/h3/robot_states', State, self.sensor_callback)
        self.emg_sub = rospy.Subscriber('hdEMG_stream', hdemg, self.get_sample)

        # Timer to send torque commands at 100 Hz
        rospy.Timer(rospy.Duration(.01),self.send_torque_cmd)
        
        # Initialize arrays for data collection
        self.first_run = True
        self.batch_ready = False
        self.sample_count = 0
        self.sample_count2 = 0
        self.sample = np.zeros((n, cst_window, 64))
        self.cst_array = np.zeros((1,n))
        self.cst_sample = [0 for i in range(n)]
        self.sample_array = [0 for i in range(n+1)]
        self.theta = 0

        self.torque_cmd = 0

        rospy.wait_for_message('/h3/robot_states', State,timeout=None)
        rospy.wait_for_message('hdEMG_stream', State,timeout=None)
        while not rospy.is_shutdown():
            # Wait for the calibration node to finish and set the flag to true 
            if rospy.get_param('calibrated') == True:
                # Re-set data arrays
                if self.first_run == True:
                    self.sample_count = 0
                    self.sample_count2 = 0
                    self.sample = np.zeros((n, cst_window, 64))
                    self.emg_norm_vals = rospy.get_param('emg_norm_vals',[1,1,1])
                    self.torque_max = rospy.get_param('Max_Torque', 35.0)

                self.first_run = False

                # rospy.loginfo("Sample: ")
                # rospy.loginfo(self.sample)

                self.torque_cmd = self.calc_torque_cst()
                                        
                r.sleep()

    def send_torque_cmd(self, event=None):
        ''' Callback for 100 Hz timer to publish torque command messages
        '''
        if rospy.get_param('calibrated') == True:

            rospy.loginfo("Torque_CMD: ")
            rospy.loginfo(self.torque_cmd)
            # if self.torque_cmd is not None:
            #     if self.torque_cmd > self.torque_max: 
            #         self.torque_cmd = self.torque_max
            #         rospy.WARN("TORQUE_CMD Exceeded Max")
            self.torque_pub.publish(self.torque_cmd)

    def get_sample(self,hdEMG):
        ''' Callback for the /hdEMG topic. 
        Organize the raw data into batches for the neural net to predict muscle activations, and then convolves the predictions into an estimation of the 
        cumulative spike train. Updates the class variable sample_array with the CST estimation and current joint angle measurement to create a 4 element 
        list for torque estimation.   
        '''

        reading = hdEMG.data.data
        num_groups = len(reading) // 64
        self.raw_emg_array.append(reading)

        samples = []
        for j in range(num_groups): #6 groups
            muscle = list(reading[64*j : 64*j + 64]) # muscle-> group 2, 3, 4
            samples.append(muscle)

        i = 0
        for m in muscles:
            if self.sample_count2 < cst_window:
                self.sample[i][self.sample_count2] = samples[m]
                self.sample_count2 += 1
            else:
                deleted = np.delete(self.sample[i], 0, axis=0)
                self.sample[i] = np.append(deleted, [np.array(samples[m])],axis=0)
                if (self.sample_count % prediction_step_size) == 0:
                    self.batch_ready = True
                    self.sample_count2 = 0
            i += 1

        if self.batch_ready:
            nueral_drive = model.predict_MUs(self.sample) #input layer (None, 40, 64)
            nueral_drive = nueral_drive.numpy()
            #print(nueral_drive)
            #print(len(sample[0]))
            cst = []
            for i in range(n):
                cst.append(np.sum(nueral_drive[:,i]))

            self.cst_array = np.concatenate([self.cst_array,np.array([cst])],axis=0)

            self.batch_ready = False

            window_hanning=[]
            window_hanning = np.hanning(np.round(0.2*512))

            cst_han1=np.array(signal.convolve(self.cst_array[:,0], window_hanning, mode='valid'))
            cst_han2=np.array(signal.convolve(self.cst_array[:,1], window_hanning, mode='valid'))
            cst_han3=np.array(signal.convolve(self.cst_array[:,2], window_hanning, mode='valid'))
            cst_han=np.concatenate([cst_han1,cst_han2,cst_han3])
            cst_han=np.array([[cst_han1], [cst_han2], [cst_han3]])
            cst_hanT=cst_han.T
            cst_han=cst_hanT.reshape(len(cst_han1),3)

            self.cst_sample = [s for s in cst_han[-1,:]]
            self.sample_array = self.cst_sample + [self.theta]

        self.sample_count += 1
        

    def sensor_callback(self,sensor_reading):
        ''' Callback for /h3/robot_states. Reads sensor messages from the h3 and saves them in class variables.
        '''
        self.theta = sensor_reading.joint_position[2]
        self.theta_dot = sensor_reading.joint_velocity[2]
        self.net_torque = sensor_reading.joint_torque_sensor[2]
        self.theta_next = self.theta + self.theta_dot * self.dt

    def calc_torque_cst(self):
        ''' Predicts torque using the model fit in the calibration node and the sample array created in the hdEMG callback.

        Returns:
            torque_cmd (float): Torque command
        '''
        coef = rospy.get_param('cst_coef')
        if len(self.cst_array) > 0:
            torque_cmd = self.f(pd.DataFrame(self.sample_array), coef)

            return torque_cmd[0]
    
    def f(self, X, betas):
        '''The model to predict torque from CST estimation

        Args:
            X (Data Frame): Data frame containing a column for each muscle and one column for joint angle        
        Returns:
            f (float[]): Array of emg predictions
        '''

        ones = pd.DataFrame({'ones': np.ones(X.iloc[1,:].size) }).T
        
        RMS_TA = X.iloc[0,:]
        RMS_GM = X.iloc[1,:]
        RMS_SOL = X.iloc[2,:]
        theta = X.iloc[3,:]

        f = (betas[0] * (RMS_TA - betas[1])).to_numpy() + (betas[2] * (RMS_TA*theta - betas[3])).to_numpy() + (betas[4] * (RMS_GM - betas[5])).to_numpy() + (betas[6] * (RMS_GM*theta - betas[7])).to_numpy() + (betas[8] * (RMS_SOL - betas[9])).to_numpy() + (betas[10] * (RMS_SOL*theta - betas[11])).to_numpy() + (betas[12]*(theta-betas[13])).to_numpy() + (betas[14] * ones).to_numpy() 

        return f[0]

if __name__ == '__main__':
    try:
        rospy.init_node('QC_node_cst', log_level=rospy.DEBUG)
        node = QC_node()
    except rospy.ROSInterruptException:
        pass
