#!/usr/bin/env python3
'''
Calibration Node

This node subscribes to the torque measurements from the exoskeleton and the high-density EMG data from the intermediate streaming node, collects and
organizes the information through 6 phases, and finally fits a model with the processed data. 

Subscribers:
    Name: /h3/robot_states Type: h3_msgs/State         Published Rate: 100 Hz 
    
    Name: /hdEMG_stream    Type: talker_listener/hdemg Published Rate: 2048 Hz 

Publishers:
    Name: /h3/right_ankle_position_controller/command Type: Float64
'''

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
from sklearn.gaussian_process import GaussianProcessRegressor
import joblib

# Parameters for trapezoid torque trajectories #
trial_length = 25 #26 #seconds
rest_time = 15 #seconds

# Neural Net Set-Up #
path = rospy.get_param("/file_dir")
model_file = path + "/src/talker_listener/" + "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model = MUdecomposer(model_file)

# Filter parameters #
nyquist = .5 * 2048.0
filter_window = [20.0/nyquist, 500.0/nyquist]

win = 40
emg_window = 100 #samples
torque_window = 25 #samples

# Parameters to Organize Raw EMG Data #
muscles = [2, 3, 4] # Channels of the inputs on the Quattrocento #MUST BE THE SAME IN BOTH FILES
n = len(muscles)
noisy_channels = [[],[],[]]

# Option to skip the calibration procedure for testing purposes
skip = False

class trial:
    def __init__(self, joint_angle, trial_length, direction, traj_shape, effort):

        self.r = rospy.Rate(2048)

        self.joint_angle = joint_angle
        self.trial_length = trial_length
        self.traj_shape = traj_shape
        self.direction = direction
        self.effort = effort

        self.MVC = 0

        self.raw_torque_array = []
        self.torque_array = []
        self.smoothed_torque_array = []

        self.raw_emg_array = []
        self.emg_array = [] 
        self.smooth_emg_win = []
        self.emg_win = []

        self.cst_array = []

    def torque_offset(self):
        if self.traj_shape == "flat":
            return np.mean(self.torque_array)

        if self.direction == "PF":
            return np.min(self.torque_array)
        elif self.direction == "DF":
            return np.max(self.torque_array) 

    def traj(self, offset):
        if self.traj_shape == "flat":
            return [0 for i in range(self.trial_length)]
        
        elif self.traj_shape == "trap":
            ''' Create a trapezoidal trajectory based on the MVC measurement '''
            min = offset
            max = self.MVC if self.direction is not "DF" else -1*self.MVC
            len = self.trial_length

            step = int(len/5)
            max = self.effort * (max - min)
            desired_traj = []
            for i in range(0, int(.5*step)):
                desired_traj.append(0)
            for i in range(int(.5*step), 2*step):
                y = (i*(max) / (step + int(.5*step))) - max/3 #2*min - max
                desired_traj.append(y)
            for i in range(2*step, 3*step):
                desired_traj.append(max)
            for i in range(3*step, int(4.5*step)):
                y = ((-i*max)/(step + int(.5*step))) + 3*max #- 3*min
                desired_traj.append(y)
            for i in range(int(4.5*step), 5*step):
                desired_traj.append(0)

            return desired_traj
        
        elif self.traj_shape == "sin":
            
            min = offset
            max = self.MVC if self.direction is not "DF" else -1*self.MVC
            len = self.trial_length

            step = int(len/5)
            max = (self.effort * (max - min))/2
            desired_traj = []

            for i in range(0, step):
                desired_traj.append((i*max / step))
            for i in range(step,4*step):
                desired_traj.append((max * np.sin(((2*np.pi)/(3*step))*(i - step))) + max)
            for i in range(4*step, 5*step):
                desired_traj.append(-i*max/step + 5*max)
            
            return desired_traj

        elif self.traj_shape == "bi-sin":
            
            min = offset
            max = self.MVC if self.direction is not "DF" else -1*self.MVC
            len = self.trial_length

            step = int(len/5)
            max = self.effort * (max - min)
            desired_traj = []

            for i in range(0, int(.5*step)):
                desired_traj.append(0)
            for i in range(int(.5*step),int(4.5*step)):
                desired_traj.append((max * np.sin(((2*np.pi)/(4*step))*(i - .5*step))))
            for i in range(int(4.5*step), 5*step):
                desired_traj.append(0)
            
            return desired_traj

class calibrate:
    def __init__(self, trials):

        self.trials = trials 

        # Set rate to the maximum of the sensor publishers
        self.r = rospy.Rate(2048)
        
        # Initialize a timer that will be used to track the subscriber rates
        self.timer = rospy.get_time()

        # Subscribers for the torque and hd-EMG publishers
        self.torque_sub = rospy.Subscriber('/h3/robot_states', State, self.torque_calib)
        self.emg_sub = rospy.Subscriber('hdEMG_stream', hdemg, self.emg_calib)

        # Publisher for position control 
        self.pos_pub = rospy.Publisher('/h3/right_ankle_position_controller/command',Float64, queue_size=0)

        # Initialize arrays for data collection
        self.start_time = rospy.Time.now().to_sec()
        self.time = []

        self.sample_count = 0
        self.offset = 0

        self.raw_torque_array = []
        self.torque_array = []
        self.smoothed_torque_array = []
        self.torque_array_for_plot = []


        self.raw_emg_array = []
        self.emg_array = [] 
        self.smooth_emg_win = []
        self.emg_win = []

        self.cst_array = []

        if skip:
            rospy.wait_for_message('/h3/robot_states', State,timeout=None)
            rospy.wait_for_message('hdEMG_stream',hdemg,timeout=None)
            rospy.set_param('emg_coef', [-8.57409162e-02, -1.00146085e+00, 2.54005172e-03, 1.60128219e-02, 8.90337001e-02, 1.58813251e+00, -3.65757650e-03, -2.47658331e-02, 5.08335815e-02, -2.35550813e-01, -1.54598354e-03, -7.65382330e-03, 5.86822916e-01, 2.87710463e+00, -1.37723825e+01])
            rospy.set_param('cst_coef', [0.613430299271461, 0.9098084781400041, 0.409857422818683, -0.20047670400913495, 0.08541811441013507, -4.42430850813377])#[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0])
            rospy.set_param('calibrated', True)
        else:
            rospy.wait_for_message('hdEMG_stream',hdemg,timeout=None)
            rospy.loginfo("Starting Baseline")
            self.data_collection()

    def max(self):
        ''' Record the maximum voluntary contraction (MVC) over a 5 second period'''

        start_index = len(self.torque_array)
        start = rospy.Time.now()
        duration = rospy.Duration.from_sec(5) 
        while (rospy.Time.now() < start + duration):
            self.r.sleep()

        MVC = np.max(np.abs(self.torque_array[start_index:])) #moving window average filter?
        return MVC

    def get_max(self):
        ''' Return the measured MVC as the average of 2 trials'''

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

        f = (betas[0] * (RMS_TA - betas[1])).to_numpy() + (betas[2] * (RMS_TA*theta - betas[3])).to_numpy() + (betas[4] * (RMS_GM - betas[5])).to_numpy() + (betas[6] * (RMS_GM*theta - betas[7])).to_numpy() + (betas[8] * (RMS_SOL - betas[9])).to_numpy() + (betas[10] * (RMS_SOL*theta - betas[11])).to_numpy() + (betas[12]*(theta-betas[13])).to_numpy() + (betas[14] * ones).to_numpy() 

        return f[0]

    def err(self, betas, df):

        prediction = self.f(df.iloc[:,df.columns != 'Torque'].T, betas)
        expected = df['Torque'].to_numpy()

        err = (expected - prediction)
        RMSE = np.sqrt(np.sum(err**2)/df.iloc[:,1].size)
            
        return RMSE

    def data_collection(self):

        for trial in self.trials:
            
            self.fig, self.axs = plt.subplots()

            # Move to 0 degrees
            rospy.loginfo("Moving to {} degrees".format(str(np.rad2deg(trial.joint_angle))))
            self.pos_pub.publish(float(trial.joint_angle))
            rospy.sleep(5)

            # Reset data arrays
            self.sample_count = 0

            self.emg_array = []
            self.emg_win = []

            self.torque_array = []
            self.smoothed_torque_array = []
            
            self.cst_array = []
            duration = rospy.Duration.from_sec(trial_length)
            
            # Measure baseline torque at rest
            rospy.loginfo("Collecting baseline")
            rospy.sleep(5)
            self.offset = np.median(self.smoothed_torque_array)
            trial.min_torque = min(self.smoothed_torque_array)
            rospy.loginfo(self.offset)

            # Measure MVC
            if trial.direction is not None:
                rospy.loginfo("Apply Max {} Torque".format(trial.direction))
                trial.MVC = self.get_max()
            else:
                trial.MVC = 2.0

            # Pause to return to resting rate
            rospy.loginfo("Rest")
            rospy.sleep(5)

            self.emg_array = []
            self.smoothed_torque_array = []
            self.torque_array_for_plot = []

            # Create a 20% Effort trapezoid trajectory
            desired_traj = trial.traj(self.offset)

            # Start real-time plot
            self.axs.plot(desired_traj, color='blue')
            self.axs.set_xlim(0, trial_length)
            self.axs.set_ylim(-1.5*trial.effort*trial.MVC, 1.5*trial.effort*trial.MVC)
            plt.pause(0.01)

            # Record indexing variables for plotting 
            start_index = len(self.torque_array_for_plot)
            start = rospy.Time.now()
            self.start_time = start.to_sec()
            self.time = []
            
            # Start data collection
            i = 0
            while (rospy.Time.now() < (start + duration)):
                
                try:
                    self.axs.plot(self.time, self.torque_array_for_plot[start_index:], color='red')
                except:
                    print("Except")
                    pass
                plt.pause(.01)
                i += 1
                self.r.sleep()

            rospy.sleep(3)

            # Save data arrays
            trial.torque_array = self.smoothed_torque_array.copy()
            trial.emg_array = np.array(self.emg_array.copy())
            trial.cst_array = np.array(self.cst_array.copy())
            
            # Prepare the next trial
            rospy.loginfo("REST")
            rospy.sleep(rest_time)
            plt.close()
        
        self.calibration()

    def calibration(self):
        self.emg_sub.unregister()
        self.torque_sub.unregister()

        print("If you just got a broken pipe error but are seeing this message, all is well")

        path = rospy.get_param("/file_dir")
        raw_torque_df = pd.DataFrame(self.raw_torque_array)
        raw_torque_df.to_csv(path + "/src/talker_listener/raw_torque.csv")

        raw_emg_df = pd.DataFrame(np.array(self.raw_emg_array))
        raw_emg_df.to_csv(path + "/src/talker_listener/raw_emg.csv")

        y = np.array(self.trials[0].torque_array - self.trials[0].torque_offset())
        for i in range(1, len(self.trials)):
            y = np.concatenate((y, np.array(self.trials[i].torque_array - self.trials[i].torque_offset())))

        emg_df = pd.DataFrame({'Torque': y})
        cst_df = pd.DataFrame({'Torque': y})
        
        for i in range(n):
            x = signal.resample(self.trials[0].emg_array[:,i], len(self.trials[0].torque_array))
            for j in range(1, len(self.trials)):
                x = np.concatenate((x, signal.resample(self.trials[j].emg_array[:,i], len(self.trials[j].torque_array))))
            
            new_column = pd.DataFrame({'Muscle '+str(muscles[i]): x})
            emg_df = pd.concat([emg_df, new_column], axis=1)
        
        angles = self.trials[0].joint_angle*np.ones(len(self.trials[0].torque_array))
        for i in range(len(self.trials)):
            angles = np.concatenate((angles, self.trials[i].joint_angle*np.ones(len(self.trials[i].torque_array))))
        
        angles = pd.DataFrame({'Angles': angles})
        emg_df = pd.concat([emg_df, angles],axis=1)
        
        emg_df = emg_df.dropna()

        b, a = signal.butter(4, .5/(.5*100), btype='lowpass')
        filtered = signal.filtfilt(b, a, emg_df, axis=0).tolist()
        filtered = np.array(filtered)

        # emg_df.iloc[:,0] = filtered[:,0]
        emg_df.iloc[:,1] = filtered[:,1]
        emg_df.iloc[:,2] = filtered[:,2]
        emg_df.iloc[:,3] = filtered[:,3]
        # emg_df.iloc[:,4] = filtered[:,4]

        path = rospy.get_param("/file_dir")
        emg_df.to_csv(path + "/src/talker_listener/test_data_EMG.csv")

        rospy.loginfo('EMG: ')
        print(emg_df)

        p0 = np.zeros(15) #np.zeros(2*(n+1))
        p0[-1] = 1
        emg_res = sp.optimize.minimize(self.err, p0, args=(emg_df)) #curve_fit(self.objective, X_emg.T, y_emg.T.to_numpy(), p0 = p0, check_finite=False)
        emg_coef = emg_res.x
        emg_coef = [float(x) for x in emg_coef]
        print('EMG Coef: ', emg_coef)

        emg_r2, emg_RMSE = self.calc_r2(emg_df['Torque'], emg_df.loc[:,emg_df.columns != 'Torque'], emg_coef)
        # rospy.loginfo("CST R^2, RMSE: ")
        # rospy.loginfo(cst_r2)
        # rospy.loginfo(cst_RMSE)
        rospy.loginfo("EMG R^2, RMSE: ")
        rospy.loginfo(emg_r2)
        rospy.loginfo(emg_RMSE)

        rospy.set_param('emg_coef',emg_coef)
        # rospy.set_param('cst_coef',cst_coef)
        rospy.set_param('calibrated', True)

    def torque_calib(self,sensor_reading):
        # print("Measured Frequency: ", 1/(rospy.get_time() - self.timer))
        # self.timer = rospy.get_time()

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
        
            self.torque_array_for_plot.append(avg-self.offset)
            self.time.append(rospy.Time.now().to_sec() - self.start_time)

    def emg_calib(self,hdEMG):
        # print(self.timer)
        # print("Measured Frequency: ", 1/(rospy.get_time() - self.timer))
        # self.timer = rospy.get_time()

        reading = hdEMG.data.data
        self.raw_emg_array.append(reading)
        num_groups = len(reading) // 64

        samples = []
        k = 0
        for j in range(num_groups):
            muscle = list(reading[64*j : 64*j + 64])
            if j in muscles:
                samples.append([m**2 for m in muscle])
                # samples.append([m**2 for ind, m in enumerate(muscle) if ind not in noisy_channels[k]])
                k += 1
        # print(samples)
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

        self.sample_count += 1


if __name__ == '__main__':
    try:
        rospy.init_node('calibration_emg', log_level=rospy.DEBUG)

        
        # Original #
        baseline = trial(0.0,25, None, "flat", 0.0)
        PF0 = trial(0.0, 25, "PF", "trap", 0.50)
        PF10 = trial(0.175, 25, "PF", "trap", 0.50)
        PFn10 = trial(-0.175, 25, "PF", "trap", 0.50)
        DF0 = trial(0.0, 25, "DF", "trap", 0.50)
        DF10 = trial(0.175, 25, "DF", "trap", 0.50)
        
        trials = [baseline, PF0, PF10, PFn10, DF0, DF10]
        

        '''
        # Angle #
        baseline = trial(0,25, None, "flat", 0.0)
        PF5 = trial(0.0873, 25, "PF", "trap", 0.50)
        PF15 = trial(0.2618, 25, "PF", "trap", 0.50)
        PFn15 = trial(-0.2618, 25, "PF", "trap", 0.50)
        DF5 = trial(0.0873, 25, "DF", "trap", 0.50)
        DF15 = trial(0.2618, 25, "DF", "trap", 0.50)
        
        trials = [baseline, PF5, PF15, PFn15, DF5, DF15]
        '''

        '''
        # Intensities #
        baseline = trial(0.0,25, None, "flat", 0.0)
        PF0 = trial(0.0, 25, "PF", "trap", 0.250)
        PF10 = trial(0.175, 25, "PF", "trap", 0.250)
        PFn10 = trial(-0.175, 25, "PF", "trap", 0.250)
        DF0 = trial(0.0, 25, "DF", "trap", 0.250)
        DF10 = trial(0.175, 25, "DF", "trap", 0.250)

        trials = [PF0, PF10, PFn10, DF0, DF10]
        '''

        '''
        # Intensities #
        baseline = trial(0.0,25, None, "flat", 0.0)
        PF0 = trial(0.0, 25, "PF", "trap", 0.750)
        PF10 = trial(0.175, 25, "PF", "trap", 0.750)
        PFn10 = trial(-0.175, 25, "PF", "trap", 0.750)
        DF0 = trial(0.0, 25, "DF", "trap", 0.750)
        DF10 = trial(0.175, 25, "DF", "trap", 0.750)

        trials = [baseline, PF0, PF10, PFn10, DF0, DF10]
        '''

        '''
        # 1D Sinusoid #
        baseline = trial(0.0,25, None, "flat", 0.0)
        PF0 = trial(0.0, 25, "PF", "sin", 0.50)
        PF10 = trial(0.175, 25, "PF", "sin", 0.50)
        PFn10 = trial(-0.175, 25, "PF", "sin", 0.50)
        DF0 = trial(0.0, 25, "DF", "sin", 0.50)
        DF10 = trial(0.175, 25, "DF", "sin", 0.50)

        trials = [PF0, PF10, PFn10, DF0, DF10]
        '''

        '''
        # 2D Sinusoid #
        baseline = trial(0.0,25, None, "flat", 0.0)
        sin10 = trial(0.175, 25, "DF", "bi-sin", 0.50)
        sin0 = trial(0.0, 25, "DF", "bi-sin", 0.50)
        sinm10 = trial(-0.175, 25, "DF", "bi-sin", 0.50)
        
        trials = [baseline, sin10, sin0, sinm10]
        '''

        # trials = [PF10]

        calibration = calibrate(trials)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass