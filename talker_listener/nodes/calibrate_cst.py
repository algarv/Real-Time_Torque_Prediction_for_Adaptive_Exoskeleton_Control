#!/usr/bin/env python3
'''
calibration_emg Node

This node subscribes to the torque measurements from the exoskeleton and the high-density EMG data from the intermediate streaming node, collects and
processes the data using a trained CNN to predict muscle activation, and finally fits a model with the processed data. 

Subscribers:
    Name: /h3/robot_states Type: h3_msgs/State         Published Rate: 100 Hz 
    
    Name: /hdEMG_stream    Type: talker_listener/hdemg Published Rate: 100 Hz 

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

# Neural Net Set-Up #
path = rospy.get_param("/file_dir")
model_file = path + "/src/talker_listener/" + "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model = MUdecomposer(model_file)

# Filter parameters #
nyquist = .5 * 510

win = 40

torque_window = 25 #samples
prediction_step_size = 20 #samples
cst_window = 40

# Parameters to Organize Raw EMG Data #
muscles = [2, 3, 4] # Channels of the inputs on the Quattrocento #MUST BE THE SAME IN BOTH FILES
n = len(muscles)
noisy_channels = [[],[],[]]

sample = np.zeros((n, cst_window, 64))

# Option to skip the calibration procedure for testing purposes
skip = False

class trial:
    ''' Object for a calibration task 
    
    Args: 
        joint_angle (float) : Position command in radians to lock exoskeleton
        trial length (float) : Number of seconds for the torque trajectory
        direction (string) : "PF" for Plantar flexion or "DF" for dorsiflexion
        traj_shape (string) : "trap" for trapezoid, "sin" for sinusoid, "bi-sin" for a bidirectional sinusoid or "flat" for baseline 0% effort 

    '''
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
        self.cst_array = []

        self.emg_start_index = 0
        self.torque_start_index = 0

        self.emg_end_index = 0
        self.torque_end_index = 0

    def torque_offset(self):
        ''' Calculate the torque offset for the trial
        '''
        print(self.torque_array)

        if self.traj_shape == "flat":
            return np.mean(self.torque_array)

        if self.direction == "PF":
            return np.min(self.torque_array)
        elif self.direction == "DF":
            return np.max(self.torque_array) 

    def traj(self, offset):
        ''' Create a reference torque trajectory of the specified shape

        Args:
            offset (float): Torque offset to subtract from the trajectory plot 
        '''
        if self.traj_shape == "flat":
            #Create a straight-line trajectory for baseline collection 

            return [0 for i in range(self.trial_length)]
        
        elif self.traj_shape == "trap":
            #Create a trapezoidal trajectory based on the MVC measurement 

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
            #Create a sinusoidal trajectory base on the MVC measurement

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
            #Create a bi-directional sinusoidal trajectory base on the MVC measurement

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
        self.cst_array = []

        if skip:
            rospy.wait_for_message('/h3/robot_states', State,timeout=None)
            rospy.wait_for_message('hdEMG_stream',hdemg,timeout=None)
            rospy.set_param('cst_coef', [-8.57409162e-02, -1.00146085e+00, 2.54005172e-03, 1.60128219e-02, 8.90337001e-02, 1.58813251e+00, -3.65757650e-03, -2.47658331e-02, 5.08335815e-02, -2.35550813e-01, -1.54598354e-03, -7.65382330e-03, 5.86822916e-01, 2.87710463e+00, -1.37723825e+01])
            rospy.set_param('calibrated', True)
        else:
            rospy.wait_for_message('hdEMG_stream',hdemg,timeout=None)
            rospy.loginfo("Starting Baseline")
            self.data_collection()

    def max(self):
        ''' Record the maximum voluntary contraction (MVC) over a 5 second period

        Returns:
            MVC (float): Maximum torque produced
        '''

        start_index = len(self.torque_array)
        start = rospy.Time.now()
        duration = rospy.Duration.from_sec(5) 
        while (rospy.Time.now() < start + duration):
            self.r.sleep()

        MVC = np.max(np.abs(self.torque_array[start_index:])) #moving window average filter?
        return MVC

    def get_max(self):
        ''' Return the measured MVC as the average of 2 trials

        Returns:
            Maximum Torque (float): Average of 2 MVCs
        '''

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
        ''' Calculate the r squared value between the predicted and measured torque

        Args:
            ydata (Data Frame): Measured torque
            xdata (Data Frame): Data from muscles and joint angle
            betas (float[]): Coefficients of best fit 
        Returns:
            r2 (float): r-squared measurement of prediction
            RMSE: root mean square error of prediction
        '''

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
        '''The model to predict torque from CST estimation

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

    def err(self, betas, df):
        '''Calculate error of predicted torque and actual torque measurements

        Args:
            betas (float[]): Coefficients of best fit
            df (Data Frame): Data frame containing measured torque, RMS emg, and joint angle
        '''

        prediction = self.f(df.iloc[:,df.columns != 'Torque'].T, betas)
        expected = df['Torque'].to_numpy()

        err = (expected - prediction)
        RMSE = np.sqrt(np.sum(err**2)/df.iloc[:,1].size)
            
        return RMSE

    def data_collection(self):
        '''Calibration procedure cycling through each task
        '''

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
            duration = rospy.Duration.from_sec(trial.trial_length)
            
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

            trial.emg_start_index = len(self.raw_emg_array)
            trial.torque_start_index = len(self.raw_torque_array)

            # Create a 20% Effort trapezoid trajectory
            desired_traj = trial.traj(self.offset)

            # Start real-time plot
            self.axs.plot(desired_traj, color='blue')
            self.axs.set_xlim(0, trial.trial_length)
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
            
            trial.emg_end_index = len(self.raw_emg_array)
            trial.torque_end_index = len(self.raw_torque_array)

            # Prepare the next trial
            rospy.loginfo("REST")
            rospy.sleep(3)
            plt.close()
        
        self.calibration()
    
    def calibration(self):
        '''Post-processing steps after data collection
        '''

        # Kill the subscribers to stop data collection
        self.emg_sub.unregister()
        self.torque_sub.unregister()

        print("Unregistered subscribers (ignore any broken pipe errors)")

        #Save raw data
        path = rospy.get_param("/file_dir")
        raw_torque_df = pd.DataFrame(self.raw_torque_array)
        raw_torque_df.to_csv(path + "/src/talker_listener/raw_torque.csv")

        raw_emg_df = pd.DataFrame(np.array(self.raw_emg_array))
        raw_emg_df.to_csv(path + "/src/talker_listener/raw_emg.csv")
        
        #Save values to help index raw data
        emg_index_list = []
        torque_index_list = []
        for trial in trials:
            emg_index_list.append([trial.emg_start_index, trial.emg_end_index])
            torque_index_list.append([trial.torque_start_index, trial.torque_end_index])

        file = open(path + "/src/talker_listener/" + "raw_data_indexes", 'w')
        for pair in emg_index_list:
            file.write("[" + str(pair[0]) + " " + str(pair[1]) + "]")
        file.write('\n')
        for pair in torque_index_list:
            file.write("[" + str(pair[0]) + " " + str(pair[1]) + "]")
        file.close()

        #Find the normalization value for each muscle and save in the parameter server
        emg_norm_vals = [0 for i in range(n)]
        for trial in self.trials:
            # Maximum value for each muscle
            for i in range(n):
                if np.max(trial.emg_array[:,i,:]) > emg_norm_vals[i]:
                    emg_norm_vals[i] = np.max(trial.emg_array[:,i,:])
        
        rospy.set_param('emg_norm_vals',[float(val) for val in emg_norm_vals]) #Convert norm values to python floats instead of numpy floats

        # Normalize each emg array, predict CSTs, and organize predictions into one data frame
        t = 0
        for trial in self.trials:
            for i in range(n):
                trial.emg_array[:,i,:] /= emg_norm_vals[i]

            data = np.array(trial.emg_array).reshape(trial.emg_array.shape[0],trial.emg_array.shape[1]*trial.emg_array.shape[2])
            print(data.shape)

            cst = self.cst_predict(data)
            print(cst.shape)

            if t == 0:
                y = np.array(trial.torque_array - trial.torque_offset())
                y = signal.resample(y, cst.shape[0])
                x = cst
                angles = trial.joint_angle*np.ones(y.shape[0])
            else:
                new_y = np.array(trial.torque_array - trial.torque_offset())
                new_y = signal.resample(new_y, cst.shape[0])
                y = np.concatenate((y, new_y))
                x = np.concatenate((x, cst))
                angles = np.concatenate((angles, trial.joint_angle*np.ones(new_y.shape[0])))
            t+=1

        cst_df = pd.DataFrame({'Torque': y})
        for i in range(n):
            new_column = pd.DataFrame({'Muscle '+str(muscles[i]): x[:,i]})
            cst_df = pd.concat([cst_df, new_column], axis=1)
        
        angles = pd.DataFrame({'Angles': angles})
        cst_df = pd.concat([cst_df, angles],axis=1)
        
        cst_df = cst_df.dropna()

        path = rospy.get_param("/file_dir")
        cst_df.to_csv(path + "/src/talker_listener/test_data_CST.csv")

        rospy.loginfo('CST: ')
        print(cst_df)

        p0 = np.zeros(15)
        p0[-1] = 1
        cst_res = sp.optimize.minimize(self.err, p0, args=(cst_df))
        cst_coef = cst_res.x
        cst_coef = [float(x) for x in cst_coef]
        print('CST Coef: ', cst_coef)

        cst_r2, cst_RMSE = self.calc_r2(cst_df['Torque'], cst_df.loc[:,cst_df.columns != 'Torque'], cst_coef)
        rospy.loginfo("CST R^2, RMSE: ")
        rospy.loginfo(cst_r2)
        rospy.loginfo(cst_RMSE)


        rospy.set_param('cst_coef',cst_coef)
        rospy.set_param('calibrated', True)

    def cst_predict(self, data):
        ''' Organize raw data into batches and feed into the trained CNN to predict motor unit activation for each muscle, and convolve over a 40ms hanning window to estimate the cumulative spike train 

        Return:
            CST (float[]): Motor unit activation convolved over a 40ms hanning window
        '''

        sample_count2 = 0
        sample_count = 0
        win = []
        cst_array = []
        batch_ready = False
        for i in range(data.shape[0]): # read each array raw
            reading = data[i,:] # raw by raw

            samples = []
            for j in range(n): #6 groups
                muscle = list(reading[64*j : 64*j + 64]) # muscle-> group 2, 3, 4
                samples.append(muscle)

            for i in range(n): # 2 3 4
                if sample_count2 < cst_window:
                    sample[i][sample_count2] = samples[i]
                    sample_count2 += 1
                else:
                    deleted = np.delete(sample[i], 0, axis=0)
                    sample[i] = np.append(deleted, [np.array(samples[i])],axis=0)
                    if (sample_count % prediction_step_size) == 0:
                        batch_ready = True
                        sample_count2 =0

            if batch_ready:
                nueral_drive = model.predict_MUs(sample) #input layer (None, 40, 64)
                nueral_drive = nueral_drive.numpy()
 
                cst = []
                for i in range(n):
                    cst.append(np.sum(nueral_drive[:,i]))
                cst_array.append(cst)
                batch_ready = False

            sample_count += 1
        
        cst_array = np.array(cst_array)

        window_hanning=[]
        window_hanning = np.hanning(np.round(0.2*512))

        cst_han1=np.array(signal.convolve(cst_array[:,0], window_hanning, mode='valid'))
        cst_han2=np.array(signal.convolve(cst_array[:,1], window_hanning, mode='valid'))
        cst_han3=np.array(signal.convolve(cst_array[:,2], window_hanning, mode='valid'))
        cst_han=np.concatenate([cst_han1,cst_han2,cst_han3])
        cst_han=np.array([[cst_han1], [cst_han2], [cst_han3]])
        cst_hanT=cst_han.T
        cst_han=cst_hanT.reshape(len(cst_han1),3)

        return np.array(cst_han)

    def torque_calib(self,sensor_reading):
        ''' Callback for the /h3/robot_states topic. Receives raw torque data and smooths it with a moving average. 
        '''

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
        ''' Callback for the /hdEMG topic. Organize the raw data into muscle groups to save only the relevant values.  
        '''

        # print(self.timer)
        # print("Measured Frequency: ", 1/(rospy.get_time() - self.timer))
        # self.timer = rospy.get_time()

        reading = hdEMG.data.data
        self.raw_emg_array.append(reading)

        num_groups = len(reading) // 64

        samples = []
        for j in range(num_groups):
            muscle = list(reading[64*j : 64*j + 64])
            if j in muscles:
                samples.append([s if (ind is not noisy_channels[muscles.index(j)]) else 0 for ind, s in enumerate(muscle) ])

        self.emg_array.append(samples)

if __name__ == '__main__':
    try:
        rospy.init_node('calibration_cst', log_level=rospy.DEBUG)

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

        trials = [PF10]

        calibration = calibrate(trials)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass