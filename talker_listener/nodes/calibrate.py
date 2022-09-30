#!/usr/bin/env python3

from cv2 import log
import rospy
import numpy as np
import scipy as sp
import pandas as pd
from scipy import signal
from scipy.optimize import curve_fit
from std_msgs.msg import Float64, Float64MultiArray
from talker_listener.qc_predict import MUdecomposer
from sklearn import linear_model as lm
import matplotlib.pyplot as plt
from rospy.core import logdebug
from h3_msgs.msg import State

trial_length = 26 #seconds
rest_time = 15 #seconds
torque_window = 10**7 #samples (100 Hz * 100 ms Window)

path = rospy.get_param("/file_dir")
#model_file = path + "\\src\\talker_listener\\best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model_file = path + "/src/talker_listener/" + "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model = MUdecomposer(model_file)

win = 40
channels = [1, 2, 3] #MUST BE THE SAME IN BOTH FILES
n = len(channels)

skip = False

class calibrate:
    def __init__(self):

        self.fig, self.axs = plt.subplots()
        self.start_time = rospy.Time.now().to_sec()
        self.sample_count = 0
        self.sample = np.zeros((n, win, 64))

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
        self.cst_array = []
        self.sample = np.zeros((n, win, 64))
        self.batch_ready = False
        self.time = []

        if skip:
            rospy.wait_for_message('/h3/robot_states', State,timeout=None)
            rospy.wait_for_message('hdEMG_stream',Float64MultiArray,timeout=None)
            rospy.set_param('emg_coef', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0])
            rospy.set_param('cst_coef', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0])
            rospy.set_param('calibrated', True)
        else:
            rospy.wait_for_message('/h3/robot_states',State,timeout=None)
            rospy.wait_for_message('hdEMG_stream',Float64MultiArray,timeout=None)
            rospy.loginfo("Starting PFO")
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
        rospy.loginfo("Go!")
        max1 = self.max()
        rospy.loginfo("MVC 1: ")
        rospy.loginfo(max1)

        rospy.loginfo("REST")
        rospy.sleep(5)

        rospy.loginfo("Go!")
        max2 = self.max()
        rospy.loginfo("MVC 2: ")
        rospy.loginfo(max2)

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
        rospy.loginfo("Moving to 0")
        self.pos_pub.publish(float(0.0))
        rospy.sleep(5)

        rospy.loginfo("Apply Max PF Torque")
        
        self.max_PF0 = self.get_max()
        rospy.set_param('max_PF0', self.max_PF0)

        desired_traj = self.make_traj_trap(0,self.max_PF0,25)

        self.axs.plot(desired_traj, color='blue')
        self.axs.set_xlim(0, 25)
        self.axs.set_ylim(0, 1.25*self.max_PF0)

        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.sample_count = 0
        self.emg_array = []
        self.cst_array = []
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
        self.PF0_cst_array = self.cst_array
        
        rospy.loginfo("REST")
        rospy.sleep(rest_time)
        plt.close()
        rospy.loginfo("Starting PF20")
        self.PF20()

    def PF20(self):
        self.fig, self.axs = plt.subplots()

        rospy.loginfo("Moving to 10 degrees")
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
        self.sample_count = 0
        self.emg_array = []
        self.cst_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)
        while (rospy.Time.now() < start + duration):
            self.axs.plot(self.time, self.smoothed_torque_array[0:len(self.time)], color='red')
            plt.pause(.01)
            self.r.sleep()

        self.PF20_torque_array = self.smoothed_torque_array
        self.PF20_emg_array = self.emg_array
        self.PF20_cst_array = self.cst_array

        rospy.loginfo("REST")
        rospy.sleep(rest_time)
        plt.close()
        rospy.loginfo("Starting PFn20")
        self.PFn20()

    def PFn20(self):
        self.fig, self.axs = plt.subplots()

        rospy.loginfo("Moving to -10 degrees")
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
        self.sample_count = 0
        self.emg_array = []
        self.cst_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)
        while (rospy.Time.now() < start + duration):
            self.axs.plot(self.time, self.smoothed_torque_array[0:len(self.time)], color='red')
            plt.pause(.01)
            self.r.sleep()

        self.PFn20_torque_array = self.smoothed_torque_array
        self.PFn20_emg_array = self.emg_array
        self.PFn20_cst_array = self.cst_array

        rospy.loginfo("REST")
        rospy.sleep(rest_time)
        plt.close()
        rospy.loginfo("Starting DF0")
        self.DF0()

    def DF0(self):
        self.fig, self.axs = plt.subplots()

        rospy.loginfo("Moving to 0 degrees")
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
        self.sample_count = 0
        self.emg_array = []
        self.cst_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)
        while (rospy.Time.now() < start + duration):
            self.axs.plot(self.time, self.smoothed_torque_array[0:len(self.time)], color='red')
            plt.pause(.01)
            self.r.sleep()

        self.DF0_torque_array = self.smoothed_torque_array
        self.DF0_emg_array = self.emg_array
        self.DF0_cst_array = self.cst_array

        rospy.loginfo("REST")
        rospy.sleep(rest_time)
        plt.close()
        rospy.loginfo("Starting DF20")
        self.DF20()

    def DF20(self):
        self.fig, self.axs = plt.subplots()

        rospy.loginfo("Moving to 20 degrees")
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
        self.sample_count = 0
        self.emg_array = []
        self.cst_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)
        while (rospy.Time.now() < start + duration):
            self.axs.plot(self.time[0:len(self.smoothed_torque_array)], self.smoothed_torque_array[0:len(self.time)], color='red')
            plt.pause(.01)
            self.r.sleep()

        self.DF20_torque_array = self.smoothed_torque_array
        self.DF20_emg_array = self.emg_array
        self.DF20_cst_array = self.cst_array

        rospy.loginfo("Starting Calibration")
        self.calibration()

    def interactionq(self, X, a, b=None, c=None, d=None, e=None, g=None, h=None, j=None, k=None, l=None, m=None):
        ones = pd.DataFrame({'ones': np.ones(len(X.columns)) }).T
        zeros = pd.DataFrame({'output': np.zeros(len(X.columns)) }).T
        f = zeros
        
        betas = [a, b, c, d, e, g, h, j, k, l, m]
        betas = [beta for beta in betas if beta is not None]
        
        for i in range(n):

            f += ((betas[i] * X.iloc[[i]]) + (betas[i + 1 + n] * X.iloc[[-1]] * X.iloc[[i]].to_numpy()).to_numpy()).to_numpy()
        
        f += (betas[n] * X.iloc[[-1]]).to_numpy()
        f += (betas[-1] * ones).to_numpy()

        print(betas)
        print(f)
        return f.to_numpy()[0]
    
    def calibration(self):
        y = np.concatenate((self.PF20_torque_array, self.PF0_torque_array, self.PFn20_torque_array, self.DF0_torque_array, self.DF20_torque_array)) #.reshape(-1,1)
        emg_df = pd.DataFrame({'Torque': y})
        cst_df = pd.DataFrame({'Torque': y})
        
        for i in range(n):
            x1 = signal.resample(self.PF20_emg_array[i], len(self.PF20_torque_array))
            x2 = signal.resample(self.PF0_emg_array[i], len(self.PF0_torque_array))
            x3 = signal.resample(self.PFn20_emg_array[i], len(self.PFn20_torque_array))
            x4 = signal.resample(self.DF0_emg_array[i], len(self.DF0_torque_array))
            x5 = signal.resample(self.DF20_emg_array[i], len(self.DF20_torque_array))

            #self.DF20_torque_array = signal.resample(self.DF20_torque_array, len(x5))

            x = np.concatenate((x1, x2, x3, x4, x5)) #.reshape(-1,1)
            new_column = pd.DataFrame({'Channel'+str(channels[i]): x})
            emg_df = pd.concat([emg_df, new_column], axis=1)
        
        angles = np.concatenate((10*np.ones(len(x1)), 0*np.ones(len(x2)), -10*np.ones(len(x3)), 0*np.ones(len(x4)), 10*np.ones(len(x5))))
        angles = pd.DataFrame({'Angles': angles})
        emg_df = pd.concat([emg_df, angles],axis=1)
        emg_df = emg_df.dropna()

        for i in range(n):
            x1 = signal.resample(self.PF20_cst_array[i], len(self.PF20_torque_array))
            x2 = signal.resample(self.PF0_cst_array[i], len(self.PF0_torque_array))
            x3 = signal.resample(self.PFn20_cst_array[i], len(self.PFn20_torque_array))
            x4 = signal.resample(self.DF0_cst_array[i], len(self.DF0_torque_array))
            x5 = signal.resample(self.DF20_cst_array[i], len(self.DF20_torque_array))

            x = np.concatenate((x1, x2, x3, x4, x5))
            new_column = pd.DataFrame({'Channel'+str(channels[i]): x})
            cst_df = pd.concat([cst_df, new_column], axis=1)
        
        angles = np.concatenate((10*np.ones(len(x1)), 0*np.ones(len(x2)), -10*np.ones(len(x3)), 0*np.ones(len(x4)), 10*np.ones(len(x5))))
        angles = pd.DataFrame({'Angles': angles})
        cst_df = pd.concat([cst_df, angles],axis=1)
        cst_df = cst_df.dropna()

        rospy.loginfo('EMG: ')
        print(emg_df)
        rospy.loginfo('CST: ')
        print(cst_df)

        # model = lm.LinearRegression()
        # X_emg = emg_df.loc[:,emg_df.columns != 'Torque']
        # y_emg = emg_df['Torque']
        # emg_res=model.fit(X_emg, y_emg) 
        # emg_int = float(emg_res.intercept_)
        # emg_coef = [float(x) for x in emg_res.coef_.tolist()]

        #model = curve_fit()
        X_emg = emg_df.loc[:,emg_df.columns != 'Torque']
        y_emg = emg_df['Torque']
        
        p0 = np.zeros(2*(n+1))
        p0[-1] = 1
        emg_res = curve_fit(self.interactionq, X_emg.T, y_emg.to_numpy(), p0 = p0, check_finite=False)
        emg_coef = emg_res[0]
        emg_coef = [float(x) for x in emg_coef]
        print('EMG Coef: ', emg_coef)
        print('EMG Cov: ', emg_res[1])

        # X_cst = cst_df.loc[:,cst_df.columns != 'Torque']
        # y_cst = cst_df['Torque']
        # cst_res=model.fit(X_cst, y_cst) 
        # cst_int = float(cst_res.intercept_)
        # cst_coef = [float(x) for x in cst_res.coef_.tolist()]

        X_cst = cst_df.loc[:,cst_df.columns != 'Torque']
        y_cst = cst_df['Torque']

        p0 = np.zeros(2*(n+1))
        p0[-1] = 1.0
        cst_res = curve_fit(self.interactionq, X_cst.T, y_cst, p0 = p0, check_finite=False)
        print(cst_res)
        cst_coef = cst_res[0]
        cst_coef = [float(x) for x in cst_coef]
        print('CST Coef: ', cst_coef)
        print('CST Cov: ', cst_res[1])

        rospy.set_param('emg_coef',emg_coef)
        rospy.set_param('cst_coef',cst_coef)
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
            else: # step size of 20 instead of 1
                deleted = np.delete(self.sample[i], 0, axis=0)
                self.sample[i] = np.append(deleted, [np.array(samples[c])],axis=0)
                self.batch_ready = True
                
            i += 1

        sample = []
        for i in range(n):
            sample.append(np.mean(self.sample[i]))
        self.emg_array.append(sample)

        # if self.sample_count % 20 == 0:
        #     self.batch_ready = True
        
        if self.batch_ready:
            nueral_drive = model.predict_MUs(self.sample)
            nueral_drive = nueral_drive.numpy()
            cst = []
            for i in range(n):
                cst.append(np.sum(nueral_drive[:,i]))
            self.cst_array.append(cst)
                
        self.sample_count += 1


if __name__ == '__main__':
    try:
        rospy.init_node('calibration', log_level=rospy.DEBUG)
        calibration = calibrate()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass