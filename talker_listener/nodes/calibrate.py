#!/usr/bin/env python3

from cv2 import log
import rospy
import numpy as np
import scipy as sp
import pandas as pd
from scipy import signal
from scipy.optimize import curve_fit
from std_msgs.msg import Float64, Float64MultiArray
from talker_listener.msg import hdemg
from talker_listener.qc_predict import MUdecomposer
from sklearn import linear_model as lm
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
from rospy.core import logdebug
from h3_msgs.msg import State
import message_filters

trial_length = 26 #seconds
rest_time = 15 #seconds
torque_window = 10 #10**7 #samples (100 Hz * 100 ms Window)

path = rospy.get_param("/file_dir")
#model_file = path + "\\src\\talker_listener\\best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model_file = path + "/src/talker_listener/" + "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model = MUdecomposer(model_file)

nyquist = .5 * 2048.0
filter_window = [20.0/nyquist, 500.0/nyquist]
win = 40
emg_avg_window = 1 #5
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
        
        self.timer = rospy.get_time()

        self.torque_sub = rospy.Subscriber('/h3/robot_states', State, self.torque_calib)
        self.emg_sub = rospy.Subscriber('hdEMG_stream', hdemg, self.emg_calib)
        # torque_sub = message_filters.Subscriber('/h3/robot_states', State)#, self.torque_calib)
        # emg_sub = message_filters.Subscriber('hdEMG_stream', hdemg)#, self.emg_calib)
        # ts = message_filters.ApproximateTimeSynchronizer([torque_sub, emg_sub], queue_size=1, slop=5)
        # ts.registerCallback(self.sensor_callback)

        self.pos_pub = rospy.Publisher('/h3/right_ankle_position_controller/command',Float64, queue_size=0)
        self.max_torque = 0
        self.torque_offset = 0
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
            rospy.wait_for_message('hdEMG_stream',hdemg,timeout=None)
            rospy.set_param('emg_coef', [1.6553566115671112, -3.3150770132099012, 1.6553566115671123, -0.3251285848589391, -0.02815857136421618, 0.01869688004015378, 0.01869688004015207, 1.6716279865773398]) #[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0])
            rospy.set_param('cst_coef', [0.613430299271461, 0.9098084781400041, 0.409857422818683, -0.20047670400913495, 0.08541811441013507, -4.42430850813377])#[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0])
            rospy.set_param('calibrated', True)
        else:
            # rospy.wait_for_message('/h3/robot_states',State,timeout=None)
            rospy.wait_for_message('hdEMG_stream',hdemg,timeout=None)
            rospy.loginfo("Starting Baseline")
            self.baseline()
    
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
        min = 0 #np.mean(self.baseline_torque_array)
        print("Min Torque: ", self.torque_offset)
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

    def baseline(self):
        rospy.loginfo("Collecting baseline")
        self.pos_pub.publish(float(0.0))

        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.sample_count = 0
        self.emg_array = []
        self.emg_win = []
        self.base_start = len(self.raw_emg_array)
        self.cst_array = []
        self.start_time = start.to_sec()
        self.time = []
        duration = rospy.Duration.from_sec(trial_length)
        i = 0

        while (rospy.Time.now() < (start + duration)):

            self.r.sleep()

        rospy.sleep(3)

        self.base_end = len(self.raw_emg_array)
        self.baseline_torque_array = self.smoothed_torque_array.copy()
        self.baseline_emg_array = np.array(self.emg_array.copy())
        self.baseline_cst_array = np.array(self.cst_array.copy())
        
        rospy.loginfo("Starting PF20")
        self.PF0()

    def PF0(self):
        rospy.loginfo("Moving to 0")
        self.pos_pub.publish(float(0.0))
        rospy.sleep(5)

        rospy.loginfo("Apply Max PF Torque")
        
        self.max_PF0 = self.get_max()
        rospy.set_param('max_PF0', self.max_PF0)

        print("Rest")
        rospy.sleep(5)

        desired_traj = self.make_traj_trap(0,self.max_PF0,25)

        self.axs.plot(desired_traj, color='blue')
        self.axs.set_xlim(0, 25)
        self.axs.set_ylim(0, 1.25*self.max_PF0)

        self.torque_array = []
        self.smoothed_torque_array = []
        self.sample_count = 0
        self.emg_array = []
        self.emg_win = []
        self.PF0_start = len(self.raw_emg_array)
        self.cst_array = []
        duration = rospy.Duration.from_sec(trial_length)
        i = 0

        print("Collecting baseline")
        rospy.sleep(5)

        start_index = len(self.smoothed_torque_array)
        start = rospy.Time.now()
        self.start_time = start.to_sec()
        self.time = []
        
        self.min_PF0 = min(self.torque_array)
        self.torque_offset = self.min_PF0

        while (rospy.Time.now() < (start + duration)):
            i += 1
            try:
                self.axs.plot(self.time, np.abs(self.smoothed_torque_array[start_index:]), color='red')
            except:
                pass
            plt.pause(.01)
            self.r.sleep()

        rospy.sleep(3)

        self.PF0_end = len(self.raw_emg_array)
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

        print("Rest")
        rospy.sleep(5)

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
        self.emg_win = []
        self.PF20_start = len(self.raw_emg_array)
        self.cst_array = []
        self.start_time = start.to_sec()
        duration = rospy.Duration.from_sec(trial_length)

        print("Collecting baseline")
        rospy.sleep(5)

        start_index = len(self.smoothed_torque_array)
        start = rospy.Time.now()
        self.start_time = start.to_sec()
        self.time = []

        self.min_PF20 = min(self.torque_array)
        self.torque_offset = self.min_PF20

        while (rospy.Time.now() < start + duration):
            try:
                self.axs.plot(self.time, np.abs(self.smoothed_torque_array[start_index:]), color='red')
            except:
                pass
            plt.pause(.01)
            self.r.sleep()

        rospy.sleep(3)

        self.PF20_end = len(self.raw_emg_array)
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

        print("Rest")
        rospy.sleep(5)

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
        self.emg_win = []
        self.PFn20_start = len(self.raw_emg_array)
        self.cst_array = []
        self.start_time = start.to_sec()
        
        duration = rospy.Duration.from_sec(trial_length)

        print("Collecting baseline")
        rospy.sleep(5)

        start_index = len(self.smoothed_torque_array)
        start = rospy.Time.now()
        self.start_time = start.to_sec()
        self.time = []

        self.min_PFn20 = min(self.torque_array)
        self.torque_offset = self.min_PFn20

        while (rospy.Time.now() < start + duration):
            try:
                self.axs.plot(self.time, np.abs(self.smoothed_torque_array[start_index:]), color='red')
            except:
                pass
            plt.pause(.01)
            self.r.sleep()

        rospy.sleep(3)

        self.PFn20_end = len(self.raw_emg_array)
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

        print("Rest")
        rospy.sleep(5)

        desired_traj = self.make_traj_trap(0,self.max_PF20,25)
        self.axs.plot(desired_traj, color='blue')
        self.axs.set_xlim(0, 26)
        self.axs.set_ylim(0, 1.25*self.max_PF0)

        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.sample_count = 0
        self.emg_array = []
        self.emg_win = []
        self.DF0_start = len(self.raw_emg_array)
        self.cst_array = []
        self.start_time = start.to_sec()
        duration = rospy.Duration.from_sec(trial_length)

        print("Collecting baseline")
        rospy.sleep(5)
        
        start_index = len(self.smoothed_torque_array)
        start = rospy.Time.now()
        self.start_time = start.to_sec()
        self.time = []

        self.min_DFn0 = min(self.torque_array)
        self.torque_offset = self.min_DFn0

        while (rospy.Time.now() < start + duration):
            try:
                self.axs.plot(self.time, np.abs(self.smoothed_torque_array[start_index:]), color='red')
            except:
                pass
            plt.pause(.01)
            self.r.sleep()

        rospy.sleep(3)

        self.DF0_end = len(self.raw_emg_array)
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

        print("Rest")
        rospy.sleep(5)

        desired_traj = self.make_traj_trap(0,self.max_DF20,25)
        self.axs.plot(desired_traj, color='blue')
        self.axs.set_xlim(0, 26)
        self.axs.set_ylim(0, 1.25*self.max_PF0)

        start = rospy.Time.now()
        self.torque_array = []
        self.smoothed_torque_array = []
        self.sample_count = 0
        self.emg_array = []
        self.emg_win = []
        self.DF20_start = len(self.raw_emg_array)
        self.cst_array = []
        self.start_time = start.to_sec()
        duration = rospy.Duration.from_sec(trial_length)

        print("Collecting baseline")
        rospy.sleep(5)

        start_index = len(self.smoothed_torque_array)
        start = rospy.Time.now()
        self.start_time = start.to_sec()
        self.time = []

        self.min_DF20 = min(self.torque_array)
        self.torque_offset = self.min_DF20

        while (rospy.Time.now() < start + duration):
            try:
                self.axs.plot(self.time[0:len(self.smoothed_torque_array)], np.abs(self.smoothed_torque_array[start_index:]), color='red')
            except:
                pass
            plt.pause(.01)
            self.r.sleep()

        rospy.sleep(3)

        self.DF20_end = len(self.raw_emg_array)
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

    def err(self, betas, df):

        prediction = self.f(df.iloc[:,df.columns != 'Torque'].T, betas)
        expected = df['Torque'].to_numpy()

        err = (expected - prediction)
        RMSE = np.sqrt(np.sum(err**2)/df.iloc[:,1].size)
            
        return RMSE

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
    
    def process_emg(self, iBase, iPF20, iPF0, iPFn20, iDF0, iDF20):
        print(iBase)

        print(len(self.raw_emg_array))
        data = np.array(self.raw_emg_array)
        print(data.shape)
        
        num_samples = len(self.baseline_torque_array) + len(self.PF20_torque_array) + len(self.PF0_torque_array) + len(self.PFn20_torque_array) + len(self.DF0_torque_array) + len(self.DF20_torque_array)
        print("Num samples: ", num_samples)
        resampled = np.zeros((num_samples,data.shape[1]))
        print("Resampling to Shape: ", resampled.shape)
        for i in range(data.shape[1]):
            x0 = signal.resample(data[iBase,i], len(self.baseline_torque_array))
            x1 = signal.resample(data[iPF20,i], len(self.PF20_torque_array))
            x2 = signal.resample(data[iPF0,i], len(self.PF0_torque_array))
            x3 = signal.resample(data[iPFn20,i], len(self.PFn20_torque_array))
            x4 = signal.resample(data[iDF0,i], len(self.DF0_torque_array))
            x5 = signal.resample(data[iDF20,i], len(self.DF20_torque_array))

            x = np.concatenate((x0, x1, x2, x3, x4, x5))
            print(i)
            resampled[:,i] = x

        print(resampled.shape)

        b, a = signal.butter(3, filter_window, btype='bandpass')
        filtered = signal.filtfilt(b, a, resampled, axis=0).tolist()
        filtered = np.array(filtered)
        print(filtered.shape)

        num_groups = filtered.shape[1] // 64

        sample_count = 0
        emg_array = []
        emg_win = []
        for i in range(filtered.shape[0]):
            reading = filtered[i,:]

            if sample_count < emg_avg_window:
                emg_win.append(reading)
            else:
                emg_win.pop(0)
                emg_win.append(reading)
            
            smoothed_reading = np.mean(emg_win, axis=0)

            samples = []
            for j in range(num_groups):
                muscle = list(smoothed_reading[64*j : 64*j + 64])
                samples.append(muscle)

            sample = []
            for j in range(n):
                sample.append(np.sqrt(np.mean([sample**2 for index,sample in enumerate(samples[muscles[j]]) if index not in noisy_channels[j]])))

            emg_array.append(sample)

            sample_count += 1

        emg_array = np.array(emg_array)

        b, a = signal.butter(4, 10/1024, btype='lowpass')
        filtered = signal.filtfilt(b, a, emg_array, axis=0).tolist()
        filtered = np.array(filtered)
        print(np.array(filtered).shape)

        return filtered #filtered[iBase,:], filtered[iPF20,:], filtered[iPF0,:], filtered[iPFn20,:], filtered[iDF0,:], filtered[iDF20,:] 

    def calibration(self):
        self.emg_sub.unregister()
        self.torque_sub.unregister()

        print("If you just got a broken pipe error but are seeing this message, all is well")

        path = rospy.get_param("/file_dir")
        raw_torque_df = pd.DataFrame(self.raw_torque_array)
        raw_torque_df.to_csv(path + "/src/talker_listener/raw_torque.csv")

        raw_emg_df = pd.DataFrame(np.array(self.raw_emg_array))
        raw_emg_df.to_csv(path + "/src/talker_listener/raw_emg.csv")

        #self.baseline_emg_array, self.PF20_emg_array, self.PF0_emg_array, self.PFn20_emg_array, self.DF0_emg_array, self.DF20_emg_array = self.process_emg(range(self.base_start, self.base_end), range(self.PF20_start, self.PF20_end), range(self.PF0_start, self.PF0_end), range(self.PFn20_start, self.PFn20_end), range(self.DF0_start, self.DF0_end), range(self.DF20_start, self.DF20_end))
        emg_array = self.process_emg(range(self.base_start, self.base_end), range(self.PF20_start, self.PF20_end), range(self.PF0_start, self.PF0_end), range(self.PFn20_start, self.PFn20_end), range(self.DF0_start, self.DF0_end), range(self.DF20_start, self.DF20_end))

        y = np.concatenate((self.baseline_torque_array - min(self.baseline_torque_array), self.PF20_torque_array - min(self.PF20_torque_array), self.PF0_torque_array - min(self.PF0_torque_array), self.PFn20_torque_array - min(self.PFn20_torque_array), self.DF0_torque_array - max(self.DF0_torque_array), self.DF20_torque_array - max(self.DF20_torque_array))) #.reshape(-1,1)
        emg_df = pd.DataFrame({'Torque': y})
        cst_df = pd.DataFrame({'Torque': y})
        
        for i in range(n):
            new_column = pd.DataFrame({'Muscle '+str(muscles[i]): emg_array[:,i]})
            emg_df = pd.concat([emg_df, new_column], axis=1)

        # print("Before: ")
        # print(self.PF20_emg_array.shape)
        # print(len(self.PF20_torque_array))

        # for i in range(n):
        #     x0 = signal.resample(self.baseline_emg_array[:,i], len(self.baseline_torque_array))
        #     x1 = signal.resample(self.PF20_emg_array[:,i], len(self.PF20_torque_array))
        #     x2 = signal.resample(self.PF0_emg_array[:,i], len(self.PF0_torque_array))
        #     x3 = signal.resample(self.PFn20_emg_array[:,i], len(self.PFn20_torque_array))
        #     x4 = signal.resample(self.DF0_emg_array[:,i], len(self.DF0_torque_array))
        #     x5 = signal.resample(self.DF20_emg_array[:,i], len(self.DF20_torque_array))

        #     # self.DF20_torque_array = signal.resample(self.DF20_torque_array, len(x5))

        #     x = np.concatenate((x0, x1, x2, x3, x4, x5)) #.reshape(-1,1)
        #     new_column = pd.DataFrame({'Muscle '+str(muscles[i]): x})
        #     emg_df = pd.concat([emg_df, new_column], axis=1)
        
        # print("PF20 Muscle 2 array after: ")
        # # print(x1)
        # print(len(x1))

        angles = np.concatenate((0*np.ones(len(self.baseline_torque_array)), 10*np.ones(len(self.PF20_torque_array)), 0*np.ones(len(self.PF0_torque_array)), -10*np.ones(len(self.PFn20_torque_array)), 0*np.ones(len(self.DF0_torque_array)), 10*np.ones(len(self.DF20_torque_array))))
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
        
        p0 = np.zeros(15) #np.zeros(2*(n+1))
        #p0[-1] = 1
        emg_res = sp.optimize.minimize(self.err, p0, args=(emg_df)) #curve_fit(self.objective, X_emg.T, y_emg.T.to_numpy(), p0 = p0, check_finite=False)
        emg_coef = emg_res.x
        emg_coef = [float(x) for x in emg_coef]
        print('EMG Coef: ', emg_coef)
        # print('EMG Cov: ', emg_res[1])

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

        rospy.set_param('emg_coef',emg_coef)
        # rospy.set_param('cst_coef',cst_coef)
        rospy.set_param('calibrated', True)

    # def sensor_callback(self,sensor_reading,hdEMG):

    #     # print("Hello?")

    #     print("TIME IN: ", rospy.get_rostime())


    #     self.raw_torque_array.append(sensor_reading.joint_torque_sensor[2])

    #     torque = sensor_reading.joint_torque_sensor[2] - self.torque_offset
        
    #     self.torque_array.append(torque)
    #     if len(self.torque_array) > 0:
    #         if (len(self.torque_array) <= torque_window):
    #             avg = np.mean(self.torque_array)
    #             self.smoothed_torque_array.append(avg)
    #         else:
    #             avg = np.sum(self.torque_array[-1*torque_window:]) / torque_window
    #             self.smoothed_torque_array.append(avg)
        
    #         self.time.append(rospy.Time.now().to_sec() - self.start_time)

    #         if self.smoothed_torque_array[-1] > self.max_torque:
    #             rospy.set_param('Max_Torque', torque)    
    #             self.max_torque = torque

    #     num_groups = len(hdEMG.data.data) // 64

    #     self.raw_emg_array.append(hdEMG.data.data)

    #     if len(self.raw_emg_array) > 27:
    #         b, a = signal.butter(4, filter_window, btype='bandpass')
    #         filtered = np.array(signal.filtfilt(b, a, self.raw_emg_array, axis=0).tolist())
    #     else:
    #         filtered = np.array(self.raw_emg_array)

    #     reading = filtered[-1,:]

    #     if len(self.emg_win) < emg_avg_window:
    #         self.emg_win.append(reading)
    #     else:
    #         self.emg_win.pop(0)
    #         self.emg_win.append(reading)
        
    #     smoothed_reading = np.mean(self.emg_win, axis=0)

    #     b, a = signal.butter(3, 10/1024, btype='lowpass')
    #     filtered = signal.filtfilt(b, a, smoothed_reading, axis=0).tolist()
    #     filtered = np.array(filtered)
    #     smoothed_reading = filtered

    #     samples = []
    #     for i in range(num_groups):
    #         muscle = list(smoothed_reading[64*i : 64*i + 64])
    #         samples.append(muscle)

    #     sample = []
    #     for i in range(n):
    #         #sample.append(np.sqrt(np.mean([j**2 for j in self.sample[i]])))
    #         sample.append(np.sqrt(np.mean([sample**2 for index,sample in enumerate(samples[muscles[i]]) if index not in noisy_channels[i]])))
    #     self.emg_array.append(sample)

    #     self.sample_count += 1


    def torque_calib(self,sensor_reading):
        torque = sensor_reading.joint_torque_sensor[2]
        
        self.raw_torque_array.append(torque)
        self.torque_array.append(torque)
        if len(self.torque_array) > 0:
            if (len(self.torque_array) <= torque_window):
                avg = np.mean(self.torque_array)
                self.smoothed_torque_array.append(avg)
            else:
                avg = np.sum(self.torque_array[-1*torque_window:]) / torque_window
                self.smoothed_torque_array.append(avg)
        
            self.time.append(rospy.Time.now().to_sec() - self.start_time)

            if self.smoothed_torque_array[-1] > self.max_torque:
                rospy.set_param('Max_Torque', torque)    
                self.max_torque = torque


    def emg_calib(self,hdEMG):

        # print(self.timer)
        # print("Measured Frequency: ", 1/(rospy.get_time() - self.timer))
        # self.timer = rospy.get_time()

        # self.batch_ready = False
        # num_groups = len(hdEMG.data.data) // 64

        self.raw_emg_array.append(hdEMG.data.data)
        # print(len(self.raw_emg_array))

        # if len(self.raw_emg_array) > 27:
        #     b, a = signal.butter(4, filter_window, btype='bandpass')
        #     filtered = np.array(signal.filtfilt(b, a, self.raw_emg_array, axis=0).tolist())
        # else:
        #     filtered = np.array(self.raw_emg_array)

        # reading = filtered[-1,:]

        # if len(self.emg_win) < emg_avg_window:
        #     self.emg_win.append(reading)
        # else:
        #     self.emg_win.pop(0)
        #     self.emg_win.append(reading)
        
        # smoothed_reading = np.mean(self.emg_win, axis=0)

        # samples = []
        # for i in range(num_groups):
        #     muscle = list(smoothed_reading[64*i : 64*i + 64])
        #     samples.append(muscle)

        
        # # torque = hdEMG.data[319]
        # # self.torque_array.append(abs(torque))
        # # if len(self.torque_array) > 0:
        # #     if (len(self.torque_array) <= torque_window):
        # #         avg = np.sum(self.torque_array[0:]) / len(self.torque_array)
        # #         self.smoothed_torque_array.append(avg)
        # #     else:
        # #         avg = np.sum(self.torque_array[-1*torque_window:-1]) / torque_window
        # #         self.smoothed_torque_array.append(avg)
        
        # #     self.time.append(rospy.Time.now().to_sec() - self.start_time)

        # #     if self.smoothed_torque_array[-1] > self.max_torque:
        # #         rospy.set_param('Max_Torque', torque)    
        # #         self.max_torque = torque

        # # i = 0
        # # for m in muscles:
        # #     if self.sample_count < win:
        # #         self.sample[i][self.sample_count] = samples[m]
        # #     else: # step size of 20 instead of 1
        # #         deleted = np.delete(self.sample[i], 0, axis=0)
        # #         self.sample[i] = np.append(deleted, [np.array(samples[m])],axis=0)
        # #         if (self.sample_count % 20) == 0:
        # #             self.batch_ready = True
                
        #     # i += 1

        # sample = []
        # for i in range(n):
        #     #sample.append(np.sqrt(np.mean([j**2 for j in self.sample[i]])))
        #     sample.append(np.sqrt(np.mean([sample**2 for index,sample in enumerate(samples[muscles[i]]) if index not in noisy_channels[i]])))
        # self.emg_array.append(sample)

        # # if self.batch_ready:
            
        # #     nueral_drive = model.predict_MUs(self.sample)
        # #     nueral_drive = nueral_drive.numpy()
        # #     cst = []
        # #     for i in range(n):
        # #         cst.append(np.sum(nueral_drive[:,i]))
        # #     self.cst_array.append(cst)
                
        # self.sample_count += 1


if __name__ == '__main__':
    try:
        rospy.init_node('calibration', log_level=rospy.DEBUG)
        calibration = calibrate()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass