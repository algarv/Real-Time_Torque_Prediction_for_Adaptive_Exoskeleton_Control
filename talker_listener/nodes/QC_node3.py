#!/usr/bin/env python

import rospy
import random
import numpy as np
import pandas as pd
from std_msgs.msg import Float64, Float64MultiArray
from talker_listener.qc_predict import MUdecomposer
from rospy.core import loginfo
from h3_msgs.msg import State
from scipy import signal
import matplotlib.pyplot as plt

path = rospy.get_param("/file_dir")
#model_file = path + "\\src\\talker_listener\\best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model_file = path + "/src/talker_listener/" + "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model = MUdecomposer(model_file)

win = 40
method = 'cst' 
adaptive = False #True
channels = [1, 2, 3] #MUST BE THE SAME IN BOTH FILES
n = len(channels)

nyquist = .5 * 2048
window = [20/nyquist, 50/nyquist]


mass = 2.37 #kg
g = -9.81 #m/s^2
l = 0.1524 #m

class QC_node:
    def __init__(self):
        r = rospy.Rate(2048) #2048 HZ for reading samples, 33 HZ for predictions  #100Hz for torque controller
        self.dt = 1.0 / 100.0
        self.torque_pub = rospy.Publisher('/h3/right_ankle_effort_controller/command', Float64, queue_size=10)
        emg_sub = rospy.Subscriber('hdEMG_stream', Float64MultiArray, self.get_sample)
        sensor_sub = rospy.Subscriber('/h3/robot_states', State, self.sensor_callback)
        
        rospy.Timer(rospy.Duration(.01),self.send_torque_cmd)
        
        self.first_run = True
        self.batch_ready = False
        self.sample_count = 0
        self.sample = np.zeros((n, win, 64))
        self.emg_array = []
        self.cst_array = []
        # self.fig, self.axs = plt.subplots()

        self.torque_cmd = 0
        rospy.wait_for_message('/h3/robot_states', State,timeout=None)
        rospy.wait_for_message('hdEMG_stream', State,timeout=None)
        while not rospy.is_shutdown():
            if rospy.get_param('calibrated') == True:
                if self.first_run == True:
                    self.sample = np.zeros((n, win, 64))
                self.first_run = False

                # rospy.loginfo("Sample: ")
                # rospy.loginfo(self.sample)

                if self.batch_ready:
                    if method == 'cst':
                        self.torque_cmd = self.calc_torque_cst()
                    
                    if method == 'emg':
                        self.torque_cmd = self.calc_torque_emg()
            
                # df = pd.DataFrame(self.emg_array)
                # df.columns = ["Muscle" + str(num) for num in channels]
                # df.plot(subplots=True, title="RMS EMG After Bandpass Filter", ax=self.axs)

                # if adaptive:
                #     self.torque_cmd = (self.torque_cmd - self.T_if) #+ (mass * g * l * np.sin(self.theta_next))

                # plt.pause(.01)
                rospy.loginfo("Torque_CMD: ")
                rospy.loginfo(self.torque_cmd)
                r.sleep()

    def send_torque_cmd(self, event=None):

        self.torque_pub.publish(self.torque_cmd)

    def get_sample(self,hdEMG):

        self.batch_ready = False
        num_groups = len(hdEMG.data) // 64

        samples = []
        for i in range(num_groups):
            muscle = list(hdEMG.data[64*i : 64*i + 64])
            samples.append(muscle)
        
        i = 0
        for c in channels:
            if self.sample_count < win:
                self.sample[i][self.sample_count] = samples[c]
            else: # step size of 20
                deleted = np.delete(self.sample[i], 0, axis=0)
                self.sample[i] = np.append(deleted, [np.array(samples[c])],axis=0)
                if (self.sample_count % 20) == 0:
                    self.batch_ready = True
                
            i += 1

        if self.batch_ready:

            sample = []
            for i in range(n):
                sample.append(np.sqrt(np.mean([j**2 for j in self.sample[i] if j is not 0])))
            self.emg_array.append(sample)
            
            nueral_drive = model.predict_MUs(self.sample)
            nueral_drive = nueral_drive.numpy()
            cst = []
            for i in range(n):
                cst.append(np.sum(nueral_drive[:,i]))
            self.cst_array.append(cst)
                
        self.sample_count += 1
        

    def sensor_callback(self,sensor_reading):
        self.theta = sensor_reading.joint_position[2]
        self.theta_dot = sensor_reading.joint_velocity[2]
        self.net_torque = sensor_reading.joint_torque_sensor[2]


        #self.T_if = self.net_torque - self.torque_cmd

        self.theta_next = self.theta + self.theta_dot * self.dt

    def calc_torque_cst(self):
        theta = self.theta
        coef = rospy.get_param('cst_coef')
        if len(self.cst_array) > 0:
            cst_sample = self.cst_array[-1]
        
            nueral_drive = model.predict_MUs(self.sample)
            nueral_drive = nueral_drive.numpy()

            torque_cmd = 0
            for i in range(n):
                torque_cmd += coef[i]*cst_sample[i] + coef[i + 1 + n]*cst_sample[i]*theta
            
            torque_cmd += theta * coef[n]
            torque_cmd += coef[-1]

            return torque_cmd

    def calc_torque_emg(self):
        theta = self.theta
        coef = rospy.get_param('emg_coef')
        if len(self.emg_array) > 0:
            emg_sample = self.emg_array[-1]
            
            torque_cmd = 0
            for i in range(n - 1):
                torque_cmd += coef[i]*emg_sample[i] + coef[i + 1 + n]*emg_sample[i]*theta

            torque_cmd += theta * coef[n]
            torque_cmd += coef[-1]
            
            return torque_cmd
        

if __name__ == '__main__':
    try:
        rospy.init_node('QC_node', log_level=rospy.DEBUG)
        node = QC_node()
    except rospy.ROSInterruptException:
        pass