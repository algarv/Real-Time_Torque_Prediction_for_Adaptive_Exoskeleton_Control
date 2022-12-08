#!/usr/bin/env python

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
#model_file = path + "\\src\\talker_listener\\best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model_file = path + "/src/talker_listener/" + "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model = MUdecomposer(model_file)

torque_max = rospy.get_param('Max_Torque', 25.0)

nyquist = .5 * 2048.0
filter_window = [20.0/nyquist, 50.0/nyquist]
win = 40
adaptive = False #True
muscles = [2, 3, 4] #MUST BE THE SAME IN BOTH FILES
n = len(muscles)
noisy_channels = [[],[],[61]]
nyquist = .5 * 2048
window = [20/nyquist, 50/nyquist]

emg_window = 300 #samples
torque_window = 50 #samples

mass = 2.37 #kg
g = -9.81 #m/s^2
l = 0.1524 #m

predictions = []

class QC_node:
    def __init__(self):
        r = rospy.Rate(2048) #2048 HZ for reading samples, 33 HZ for predictions  #100Hz for torque controller
        self.dt = 1.0 / 100.0
        self.torque_pub = rospy.Publisher('/h3/right_ankle_effort_controller/command', Float64, queue_size=10)
        self.sensor_sub = rospy.Subscriber('/h3/robot_states', State, self.sensor_callback)
        self.emg_sub = rospy.Subscriber('hdEMG_stream', hdemg, self.get_sample)
        # torque_sub = message_filters.Subscriber('/h3/robot_states', State)#, self.torque_calib)
        # emg_sub = message_filters.Subscriber('hdEMG_stream', hdemg)#, self.emg_calib)
        # ts = message_filters.ApproximateTimeSynchronizer([torque_sub, emg_sub], queue_size=25, slop=0.01)
        # ts.registerCallback(self.sensor_callback)
        
        rospy.Timer(rospy.Duration(.01),self.send_torque_cmd)
        
        self.first_run = True
        self.batch_ready = False
        self.sample_count = 0
        self.sample = np.zeros((n, win, 64))
        self.emg_array = []
        self.raw_emg_array = []
        self.emg_win = []
        self.smooth_emg_win = []
        self.theta = 0
        # self.fig, self.axs = plt.subplots()

        self.torque_cmd = 0
        rospy.wait_for_message('/h3/robot_states', State,timeout=None)
        rospy.wait_for_message('hdEMG_stream', State,timeout=None)
        while not rospy.is_shutdown():
            if rospy.get_param('calibrated') == True:
                if self.first_run == True:
                    self.sample_count = 0
                    self.sample = np.zeros((n, win, 64))
                    # self.gpr = joblib.load(path + "/src/talker_listener/emg_gpr")

                self.first_run = False

                # rospy.loginfo("Sample: ")
                # rospy.loginfo(self.sample)
    
                self.torque_cmd = self.calc_torque_emg()
        
                predictions.append(self.torque_cmd)
                # df = pd.DataFrame(self.emg_array)
                # df.columns = ["Muscle" + str(num) for num in muscles]
                # df.plot(subplots=True, title="RMS EMG After Bandpass Filter", ax=self.axs)

                # if adaptive:
                #     self.torque_cmd = (self.torque_cmd - self.T_if) #+ (mass * g * l * np.sin(self.theta_next))

                # plt.pause(.01)
                # predictions_df = pd.DataFrame(predictions)
                # predictions_df.to_csv(path + "/src/talker_listener/predictions.csv")
                
                r.sleep()

    def send_torque_cmd(self, event=None):

        if rospy.get_param('calibrated') == True:
            rospy.loginfo("Torque_CMD: ")
            rospy.loginfo(self.torque_cmd)
            # if self.torque_cmd is not None:
            #     if self.torque_cmd > torque_max: 
            #         self.torque_cmd = torque_max
            #         rospy.WARN("TORQUE_CMD Exceeded Max")

            self.torque_pub.publish(self.torque_cmd)

    def get_sample(self,hdEMG):

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
        # print("HELLO")
        self.theta = sensor_reading.joint_position[2]
        self.theta_dot = sensor_reading.joint_velocity[2]
        self.net_torque = sensor_reading.joint_torque_sensor[2]
        self.theta_next = self.theta + self.theta_dot * self.dt
        #self.T_if = self.net_torque - self.torque_cmd

        # self.batch_ready = True
        # num_groups = len(hdEMG.data.data) // 64

        # self.raw_emg_array.append(hdEMG.data.data)
        
        # samples = []
        # for i in range(num_groups):
        #     muscle = list(hdEMG.data.data[64*i : 64*i + 64])
        #     samples.append(muscle)

        # sample = []
        # for i in range(n):
        #     sample.append(np.sqrt(np.mean([s**2 for index,s in enumerate(samples[muscles[i]]) if index not in noisy_channels[i]])))
        
        # self.emg_array.append(sample)
        
        # self.sample_array = sample + [self.theta]

    def calc_torque_emg(self):
        coef = rospy.get_param('emg_coef')
        # print(len(self.emg_array))
        if len(self.emg_array) > 0:
            # print(self.sample_array)
            #torque_cmd = self.gpr.predict(np.array([self.sample_array]))
            torque_cmd = self.f(pd.DataFrame(self.sample_array), coef)
            
            # torque_cmd = 0
            # for i in range(n):
            #     torque_cmd += coef[i]*emg_sample[i] + coef[i + 1 + n]*emg_sample[i]*theta

            # torque_cmd += theta * coef[n]
            # torque_cmd += coef[-1]

            return -1*torque_cmd[0]
        
    def f(self, X, betas):
        ones = pd.DataFrame({'ones': np.ones(X.iloc[1,:].size) }).T
        zeros = pd.DataFrame({'output': np.zeros(X.iloc[1,:].size) }).T
        
        RMS_TA = X.iloc[0,:]
        RMS_GM = X.iloc[1,:]
        RMS_SOL = X.iloc[2,:]
        theta = X.iloc[3,:]

        # for i in range(n):
        #     f += ((betas[i] * X.iloc[i,:]) + (betas[i + 1 + n] * X.iloc[-1, :] * X.iloc[i,:].to_numpy()).to_numpy()).to_numpy()

        # f += (betas[n] * X.iloc[-1,:]).to_numpy()
        # f += (betas[-1] * ones).to_numpy()

        f = (betas[0] * (RMS_TA - betas[1])).to_numpy() + (betas[2] * (RMS_TA*theta - betas[3])).to_numpy() + (betas[4] * (RMS_GM - betas[5])).to_numpy() + (betas[6] * (RMS_GM*theta - betas[7])).to_numpy() + (betas[8] * (RMS_SOL - betas[9])).to_numpy() + (betas[10] * (RMS_SOL*theta - betas[11])).to_numpy() + (betas[12]*(theta-betas[13])).to_numpy() + (betas[14] * ones).to_numpy() 

        return f[0]

if __name__ == '__main__':
    try:
        rospy.init_node('QC_node_emg', log_level=rospy.DEBUG)
        node = QC_node()
    except rospy.ROSInterruptException:
        pass
