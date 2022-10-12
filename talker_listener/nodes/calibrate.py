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
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
from rospy.core import logdebug
from h3_msgs.msg import State

trial_length = 26 #seconds
rest_time = 15 #seconds
torque_window = 10 #10**7 #samples (100 Hz * 100 ms Window)

path = rospy.get_param("/file_dir")
#model_file = path + "\\src\\talker_listener\\best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model_file = path + "/src/talker_listener/" + "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model = MUdecomposer(model_file)

nyquist = .5 * 2048.0
filter_window = [20.0/nyquist, 50.0/nyquist]
win = 40
emg_avg_window = 100
muscles = [2, 3, 4] #MUST BE THE SAME IN BOTH FILES
n = len(muscles)

#noisy_channels = [[1, 15], [40], [16, 17, 30, 56, 62]]
noisy_channels = [[],[],[61]] #[[0, 14], [39], [15, 16, 29, 55, 61]]

skip = False

class calibrate:
    def __init__(self):

        self.fig, self.axs = plt.subplots()
        self.start_time = rospy.Time.now().to_sec()
        self.sample_count = 0
        self.sample = np.zeros((n, win, 64))

        self.r = rospy.Rate(2048)
        torque_sub = rospy.Subscriber('/h3/robot_states', State, self.torque_calib)
        emg_sub = rospy.Subscriber('hdEMG_stream', Float64MultiArray, self.emg_calib)
        self.pos_pub = rospy.Publisher('/h3/right_ankle_position_controller/command',Float64, queue_size=0)
        self.max_torque = 0
        self.torque_array = []
        self.smoothed_torque_array = []
        self.PF0_array = []
        self.PF10_array = []
        self.DF0_array = []
        self.DF10_array = []
        self.emg_array = []
        self.emg_win = []
        self.cst_array = []
        self.raw_torque_array = []
        self.raw_emg_array = []
        self.sample = np.zeros((n, win, 64))
        self.batch_ready = False
        self.time = []

        if skip:
            rospy.wait_for_message('/h3/robot_states', State,timeout=None)
            rospy.wait_for_message('hdEMG_stream',Float64MultiArray,timeout=None)
            rospy.set_param('emg_coef', [0.3363840542457709, -0.3174651343625933, -0.46532159159751196, 0.0509680862794715, -0.046112467433775936, 0.46658086763974144]) #[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0])
            rospy.set_param('cst_coef', [0.613430299271461, 0.9098084781400041, 0.409857422818683, -0.20047670400913495, 0.08541811441013507, -4.42430850813377])#[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0])
            rospy.set_param('calibrated', True)
        else:
            # rospy.wait_for_message('/h3/robot_states',State,timeout=None)
            rospy.wait_for_message('hdEMG_stream',Float64MultiArray,timeout=None)
            rospy.loginfo("Starting PFO")
            self.PF0()
    
    def max(self):

        start_index = len(self.torque_array)
        start = rospy.Time.now()
        duration = rospy.Duration.from_sec(5) 
        while (rospy.Time.now() < start + duration):
            self.r.sleep()

        MVC = np.max(np.abs(self.torque_array[start_index:])) #moving window average filter?
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
        # desired_traj = [0, 0, 0]
        # step =  .20 * (max-min) / 5
        # prev = 0
        # for i in range(5):
        #     desired_traj.append(prev)
        #     prev += step
        # for i in range(10):
        #     desired_traj.append(.20 * self.max_PF0)
        # prev = desired_traj[-1]
        # for i in range(5):
        #     desired_traj.append(prev)
        #     prev -= step

        # desired_traj.append(0)
        # desired_traj.append(0)
        # desired_traj.append(0)
        step = int(len/5)
        max = .2 * max
        # max += 2.0
        # min = 2.0
        desired_traj = []
        for i in range(0, step):
            desired_traj.append(min)
        for i in range(step, 2*step):
            y = i * ((max - min) / step) + 2*min - max
            desired_traj.append(y)
        for i in range(2*step, 3*step):
            desired_traj.append(max)
        for i in range(3*step, 4*step):
            y = -1*i*((max-min)/step) + 4*max - 3*min
            desired_traj.append(y)
        for i in range(4*step, 5*step):
            desired_traj.append(min)

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

        rospy.sleep(3)

        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.sample_count = 0
        self.emg_array = []
        self.emg_win = []
        self.cst_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)
        i = 0

        while (rospy.Time.now() < (start + duration)):
            i += 1
            try:
                self.axs.plot(self.time, np.abs(self.smoothed_torque_array[0:len(self.time)]), color='red')
            except:
                pass
            plt.pause(.01)
            self.r.sleep()

        rospy.sleep(3)

        self.PF0_torque_array = self.smoothed_torque_array.copy()
        self.PF0_emg_array = np.array(self.emg_array.copy())
        self.PF0_cst_array = np.array(self.cst_array.copy())
        
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

        rospy.sleep(3)

        #self.calib_index = len(self.torque_array)
        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.sample_count = 0
        self.emg_array = []
        self.emg_win = []
        self.cst_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)

        while (rospy.Time.now() < start + duration):
            try:
                self.axs.plot(self.time, np.abs(self.smoothed_torque_array[0:len(self.time)]), color='red')
            except:
                pass
            plt.pause(.01)
            self.r.sleep()

        rospy.sleep(3)

        self.PF20_torque_array = self.smoothed_torque_array.copy()
        self.PF20_emg_array = np.array(self.emg_array.copy())
        self.PF20_cst_array = np.array(self.cst_array.copy())

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

        rospy.sleep(3)

        #self.calib_index = len(self.torque_array)
        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.sample_count = 0
        self.emg_array = []
        self.emg_win = []
        self.cst_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)

        while (rospy.Time.now() < start + duration):
            try:
                self.axs.plot(self.time, np.abs(self.smoothed_torque_array[0:len(self.time)]), color='red')
            except:
                pass
            plt.pause(.01)
            self.r.sleep()

        rospy.sleep(3)

        self.PFn20_torque_array = self.smoothed_torque_array.copy()
        self.PFn20_emg_array = np.array(self.emg_array.copy())
        self.PFn20_cst_array = np.array(self.cst_array.copy())

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

        rospy.sleep(3)

        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.sample_count = 0
        self.emg_array = []
        self.emg_win = []
        self.cst_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)

        while (rospy.Time.now() < start + duration):
            try:
                self.axs.plot(self.time, np.abs(self.smoothed_torque_array[0:len(self.time)]), color='red')
            except:
                pass
            plt.pause(.01)
            self.r.sleep()

        rospy.sleep(3)

        self.DF0_torque_array = self.smoothed_torque_array.copy()
        self.DF0_emg_array = np.array(self.emg_array.copy())
        self.DF0_cst_array = np.array(self.cst_array.copy())

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

        rospy.sleep(3)

        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.sample_count = 0
        self.emg_array = []
        self.emg_win = []
        self.cst_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)

        while (rospy.Time.now() < start + duration):
            try:
                self.axs.plot(self.time[0:len(self.smoothed_torque_array)], np.abs(self.smoothed_torque_array[0:len(self.time)]), color='red')
            except:
                pass
            plt.pause(.01)
            self.r.sleep()

        rospy.sleep(3)

        self.DF20_torque_array = self.smoothed_torque_array.copy()
        self.DF20_emg_array = np.array(self.emg_array.copy())
        self.DF20_cst_array = np.array(self.cst_array.copy())

        rospy.loginfo("Starting Calibration")
        self.calibration()

    def calc_r2(self, ydata, xdata, betas):

        ydata = ydata.T
        xdata = xdata.T

        err = (ydata - self.f(xdata, betas)).to_numpy()
        SSR = np.sum(err**2)
        norm = (ydata - np.mean(ydata)).to_numpy()
        SSN = np.sum(norm**2)

        RMSE = np.sqrt(SSR/xdata.iloc[1,:].size)
        r2 = 1 - (SSR / SSN)

        return r2, RMSE
    
    def f(self, X, betas):
        ones = pd.DataFrame({'ones': np.ones(X.iloc[1,:].size) }).T
        zeros = pd.DataFrame({'output': np.zeros(X.iloc[1,:].size) }).T
        f = zeros

        for i in range(n):

            f += ((betas[i] * X.iloc[1,:]) + (betas[i + 1 + n] * X.iloc[-1, :] * X.iloc[1,:].to_numpy()).to_numpy()).to_numpy()
        
        f += (betas[n] * X.iloc[-1,:]).to_numpy()
        f += (betas[-1] * ones).to_numpy()

        return f.to_numpy()[0]

    def objective(self, X, a, b=None, c=None, d=None, e=None, g=None, h=None, j=None, k=None, l=None, m=None, o=None, p=None, q=None, r=None):
        # ones = pd.DataFrame({'ones': np.ones(len(X.columns)) }).T
        # zeros = pd.DataFrame({'output': np.zeros(len(X.columns)) }).T
        # f = zeros
        
        betas = [a, b, c, d, e, g, h, j, k, l, m, o, p, q, r]
        betas = [beta for beta in betas if beta is not None]
        
        # for i in range(n):

        #     f += ((betas[i] * X.iloc[[i]]) + (betas[i + 1 + n] * X.iloc[[-1]] * X.iloc[[i]].to_numpy()).to_numpy()).to_numpy()
        
        # f += (betas[n] * X.iloc[[-1]]).to_numpy()
        # f += (betas[-1] * ones).to_numpy()

        return self.f(X, betas) #f.to_numpy()[0]
    
    def calibration(self):
        path = rospy.get_param("/file_dir")
        raw_torque_df = pd.DataFrame(self.raw_torque_array)
        raw_torque_df.to_csv(path + "/src/talker_listener/raw_torque.csv")

        raw_emg_df = pd.DataFrame(self.raw_emg_array)
        raw_emg_df.to_csv(path + "/src/talker_listener/raw_emg.csv")
        
        y = np.concatenate((self.PF20_torque_array, self.PF0_torque_array, self.PFn20_torque_array, self.DF0_torque_array, self.DF20_torque_array)) #.reshape(-1,1)
        emg_df = pd.DataFrame({'Torque': y})
        cst_df = pd.DataFrame({'Torque': y})
        
        print("Before: ")
        print(self.PF20_emg_array.shape)
        print(len(self.PF20_torque_array))

        for i in range(n):
            x1 = signal.resample(self.PF20_emg_array[:,i], len(self.PF20_torque_array))
            x2 = signal.resample(self.PF0_emg_array[:,i], len(self.PF0_torque_array))
            x3 = signal.resample(self.PFn20_emg_array[:,i], len(self.PFn20_torque_array))
            x4 = signal.resample(self.DF0_emg_array[:,i], len(self.DF0_torque_array))
            x5 = signal.resample(self.DF20_emg_array[:,i], len(self.DF20_torque_array))

            #self.DF20_torque_array = signal.resample(self.DF20_torque_array, len(x5))

            x = np.concatenate((x1, x2, x3, x4, x5)) #.reshape(-1,1)
            new_column = pd.DataFrame({'Muscle '+str(muscles[i]): x})
            emg_df = pd.concat([emg_df, new_column], axis=1)
        
        print("PF20 Muscle 2 array after: ")
        # print(x1)
        print(len(x1))

        angles = np.concatenate((10*np.ones(len(x1)), 0*np.ones(len(x2)), -10*np.ones(len(x3)), 0*np.ones(len(x4)), 10*np.ones(len(x5))))
        angles = pd.DataFrame({'Angles': angles})
        emg_df = pd.concat([emg_df, angles],axis=1)
        emg_df = emg_df.dropna()

        path = rospy.get_param("/file_dir")
        emg_df.to_csv(path + "/src/talker_listener/test_data_EMG.csv")

        # for i in range(n):
        #     x1 = signal.resample(self.PF20_cst_array[i], len(self.PF20_torque_array))
        #     x2 = signal.resample(self.PF0_cst_array[i], len(self.PF0_torque_array))
        #     x3 = signal.resample(self.PFn20_cst_array[i], len(self.PFn20_torque_array))
        #     x4 = signal.resample(self.DF0_cst_array[i], len(self.DF0_torque_array))
        #     x5 = signal.resample(self.DF20_cst_array[i], len(self.DF20_torque_array))

        #     x = np.concatenate((x1, x2, x3, x4, x5))
        #     new_column = pd.DataFrame({'Muscle'+str(muscles[i]): x})
        #     cst_df = pd.concat([cst_df, new_column], axis=1)
        
        # angles = np.concatenate((10*np.ones(len(x1)), 0*np.ones(len(x2)), -10*np.ones(len(x3)), 0*np.ones(len(x4)), 10*np.ones(len(x5))))
        # angles = pd.DataFrame({'Angles': angles})
        # cst_df = pd.concat([cst_df, angles],axis=1)
        # cst_df = cst_df.dropna()

        # path = rospy.get_param("/file_dir")
        # cst_df.to_csv(path + "/src/talker_listener/test_data_CST.csv")

        cst_df, cst_test_df = train_test_split(cst_df, test_size=.20)
        emg_df, emg_test_df = train_test_split(emg_df, test_size=.20)

        rospy.loginfo('EMG: ')
        print(emg_df)
        # rospy.loginfo('CST: ')
        # print(cst_df)

        # model = lm.LinearRegression()
        # X_emg = emg_df.loc[:,emg_df.columns != 'Torque']
        # y_emg = emg_df['Torque']
        # emg_res=model.fit(X_emg, y_emg) 
        # emg_int = float(emg_res.intercept_)
        # emg_coef = [float(x) for x in emg_res.coef_.tolist()]

        X_emg = emg_df.loc[:,emg_df.columns != 'Torque']
        y_emg = emg_df['Torque']
        
        p0 = np.zeros(2*(n+1))
        p0[-1] = 1
        emg_res = curve_fit(self.objective, X_emg.T, y_emg.T.to_numpy(), p0 = p0, check_finite=False)
        emg_coef = emg_res[0]
        emg_coef = [float(x) for x in emg_coef]
        print('EMG Coef: ', emg_coef)
        print('EMG Cov: ', emg_res[1])

        # X_cst = cst_df.loc[:,cst_df.columns != 'Torque']
        # y_cst = cst_df['Torque']
        # cst_res=model.fit(X_cst, y_cst) 
        # cst_int = float(cst_res.intercept_)
        # cst_coef = [float(x) for x in cst_res.coef_.tolist()]

        # X_cst = cst_df.loc[:,cst_df.columns != 'Torque']
        # y_cst = cst_df['Torque']

        # p0 = np.zeros(2*(n+1))
        # p0[-1] = 1.0
        # cst_res = curve_fit(self.objective, X_cst.T, y_cst, p0 = p0, check_finite=False)
        # print(cst_res)
        # cst_coef = cst_res[0]
        # cst_coef = [float(x) for x in cst_coef]
        # print('CST Coef: ', cst_coef)
        # print('CST Cov: ', cst_res[1])

        # cst_r2, cst_RMSE = self.calc_r2(cst_test_df['Torque'], cst_test_df.loc[:,cst_test_df.columns != 'Torque'], cst_coef)
        emg_r2, emg_RMSE = self.calc_r2(emg_test_df['Torque'], emg_test_df.loc[:,emg_test_df.columns != 'Torque'], emg_coef)
        # rospy.loginfo("CST R^2, RMSE: ")
        # rospy.loginfo(cst_r2)
        # rospy.loginfo(cst_RMSE)
        rospy.loginfo("EMG R^2, RMSE: ")
        rospy.loginfo(emg_r2)
        rospy.loginfo(emg_RMSE)

        plt.plot(emg_df)
        plt.show()

        rospy.set_param('emg_coef',emg_coef)
        # rospy.set_param('cst_coef',cst_coef)
        rospy.set_param('calibrated', True)

    def torque_calib(self,sensor_reading):
        torque = sensor_reading.joint_torque_sensor[2]
        
        self.raw_torque_array.append(torque)
        self.torque_array.append(torque)
        if len(self.torque_array) > 0:
            if (len(self.torque_array) <= torque_window):
                avg = np.sum(self.torque_array[0:]) / len(self.torque_array)
                self.smoothed_torque_array.append(avg)
            else:
                avg = np.sum(self.torque_array[-1*torque_window:]) / torque_window
                self.smoothed_torque_array.append(avg)
        
            self.time.append(rospy.Time.now().to_sec() - self.start_time)

            if self.smoothed_torque_array[-1] > self.max_torque:
                rospy.set_param('Max_Torque', torque)    
                self.max_torque = torque

    def emg_calib(self,hdEMG):
        sample_ready = False
        self.batch_ready = False
        num_groups = len(hdEMG.data) // 64

        self.raw_emg_array.append(hdEMG.data)

        if len(self.raw_emg_array) > 27:
            b, a = signal.butter(4, filter_window, btype='bandpass')
            filtered = np.array(signal.filtfilt(b, a, self.raw_emg_array, axis=0).tolist())
        else:
            filtered = np.array(self.raw_emg_array)

        reading = filtered[-1,:]

        if len(self.emg_win) < emg_avg_window:
            self.emg_win.append(reading)
        else:
            self.emg_win.pop(0)
            self.emg_win.append(reading)
            sample_ready = True
        
        if sample_ready:
            smoothed_reading = np.mean(self.emg_win, axis=0)
        else:
            smoothed_reading = reading

        samples = []
        for i in range(num_groups):
            muscle = list(smoothed_reading[64*i : 64*i + 64])
            samples.append(muscle)
        
        # torque = hdEMG.data[319]
        # self.torque_array.append(abs(torque))
        # if len(self.torque_array) > 0:
        #     if (len(self.torque_array) <= torque_window):
        #         avg = np.sum(self.torque_array[0:]) / len(self.torque_array)
        #         self.smoothed_torque_array.append(avg)
        #     else:
        #         avg = np.sum(self.torque_array[-1*torque_window:-1]) / torque_window
        #         self.smoothed_torque_array.append(avg)
        
        #     self.time.append(rospy.Time.now().to_sec() - self.start_time)

        #     if self.smoothed_torque_array[-1] > self.max_torque:
        #         rospy.set_param('Max_Torque', torque)    
        #         self.max_torque = torque

        i = 0
        for m in muscles:
            if self.sample_count < win:
                self.sample[i][self.sample_count] = samples[m]
            else: # step size of 20 instead of 1
                deleted = np.delete(self.sample[i], 0, axis=0)
                self.sample[i] = np.append(deleted, [np.array(samples[m])],axis=0)
                if (self.sample_count % 20) == 0:
                    self.batch_ready = True
                
            i += 1

        sample = []
        for i in range(n):
            #sample.append(np.sqrt(np.mean([j**2 for j in self.sample[i]])))
            sample.append(np.sqrt(np.mean([sample**2 for index,sample in enumerate(samples[muscles[i]]) if index not in noisy_channels[i]])))
        self.emg_array.append(sample)

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