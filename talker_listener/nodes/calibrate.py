#!/usr/bin/env python

import rospy 
import numpy as np
import scipy as sp
from scipy import signal
from std_msgs.msg import Float64, Float64MultiArray
from sklearn import linear_model as lm
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rospy.core import logdebug
from h3_msgs.msg import State

max_torque = 0
trial_length = 15 #seconds

def torque_calib(sensor_reading):
    global torque_array, start_time
    torque = sensor_reading.joint_torque_sensor[2]
    torque_array.append(torque)
    time.append(rospy.Time.now().to_sec() - start_time)
    if torque > max_torque:
        rospy.set_param('Max_Torque', torque)    

def emg_calib(hdEMG):
    global emg_array
    sample = np.mean(hdEMG.data)
    emg_array.append(sample) #pick specific value?

def main():
    rospy.init_node('calibration', log_level=rospy.DEBUG)

    global plot, fig, axs, start_time
    fig, axs = plt.subplots()
    axs.set_xlim(0,trial_length)
    axs.set_ylim(-10,10)
    start_time = rospy.Time.now().to_sec()
    desired_traj = np.concatenate((np.linspace(0,5,5),5*np.ones(5),np.linspace(5,0,6)))
    axs.plot(desired_traj, color='blue')

    torque_sub = rospy.Subscriber('/h3/robot_states', State, torque_calib)
    emg_sub = rospy.Subscriber('hdEMG_stream', Float64MultiArray, emg_calib)
    r = rospy.Rate(100)

    global torque_array
    global emg_array
    global time
    torque_array = []
    emg_array = []
    time = []

    start = rospy.Time.now()
    duration = rospy.Duration.from_sec(trial_length) 
    while (rospy.Time.now() < start + duration):
        # logdebug(time)
        # logdebug(torque_array)
        # logdebug(emg_array)
        axs.plot(time, torque_array, color='red')
        plt.pause(.01)
        r.sleep()

    y = np.array(torque_array).reshape(-1,1)
    x = signal.resample(emg_array, len(y))
    x = np.array(x).reshape(-1,1)
    model = lm.LinearRegression()
    res=model.fit(x,y)
    intercept = res.intercept_[0]
    slope = res.coef_[0][0]
    intercept = float(intercept)
    slope = float(slope)
    logdebug(intercept)
    logdebug(slope)
    rospy.set_param('intercept',intercept)
    rospy.set_param('slope',slope)
    rospy.set_param('calibrated', True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass