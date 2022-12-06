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

nyquist = .5 * 512
filter_window = [20.0/nyquist, 50.0/nyquist]
win = 40
method = 'cst' 
adaptive = False #True
muscles = [2, 3, 4] #MUST BE THE SAME IN BOTH FILES
n = len(muscles)
noisy_channels = [[],[],[]]
window = [20/nyquist, 50/nyquist]

emg_window = 300 #samples
torque_window = 50 #samples
cst_window = 40
prediction_step_size = 20

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
        self.sample_count2 = 0
        self.sample = np.zeros((n, cst_window, 64))
        self.emg_array = []
        self.cst_array = np.zeros((1,n))
        self.raw_emg_array = []
        self.emg_win = []
        self.smooth_emg_win = []
        self.cst_sample = [0 for i in range(n)]
        self.sample_array = [0 for i in range(n+1)]
        self.theta = 0
        # self.fig, self.axs = plt.subplots()

        self.torque_cmd = 0
        rospy.wait_for_message('/h3/robot_states', State,timeout=None)
        rospy.wait_for_message('hdEMG_stream', State,timeout=None)
        while not rospy.is_shutdown():
            self.calibrated = rospy.get_param('calibrated')
            if self.calibrated:
                if self.first_run == True:
                    self.sample_count = 0
                    self.sample_count2 = 0
                    self.sample = np.zeros((n, win, 64))
                    self.emg_norm_vals = rospy.get_param('emg_norm_vals',[1,1,1])
                    self.torque_max = rospy.get_param('Max_Torque', 35.0)
                    # self.gpr = joblib.load(path + "/src/talker_listener/emg_gpr")

                self.first_run = False

                # rospy.loginfo("Sample: ")
                # rospy.loginfo(self.sample)

                self.torque_cmd = self.calc_torque_cst()
                        
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
            #     if self.torque_cmd > self.torque_max: 
            #         self.torque_cmd = self.torque_max
            #         rospy.WARN("TORQUE_CMD Exceeded Max")
            self.torque_pub.publish(self.torque_cmd)

    def get_sample(self,hdEMG):

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

        if self.batch_ready and self.calibrated:
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
        self.theta = sensor_reading.joint_position[2]
        self.theta_dot = sensor_reading.joint_velocity[2]
        self.net_torque = sensor_reading.joint_torque_sensor[2]
        self.theta_next = self.theta + self.theta_dot * self.dt

    def calc_torque_cst(self):
        coef = rospy.get_param('cst_coef')
        if len(self.cst_array) > 0:
            torque_cmd = self.f(pd.DataFrame(self.sample_array), coef)

            return torque_cmd[0]
    
    def f(self, X, betas):
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
