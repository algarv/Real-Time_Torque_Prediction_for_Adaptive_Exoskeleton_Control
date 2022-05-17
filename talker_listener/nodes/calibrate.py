#!/usr/bin/env python

import rospy 
import numpy as np
import scipy as sp
from scipy import signal
from std_msgs.msg import Float64, Float64MultiArray
from sklearn import linear_model as lm
from rospy.core import logdebug
#import State.msg
from h3_msgs.msg import State

max_torque = 0
trial_length = 8 #seconds

def torque_calib(sensor_reading):
    global torque_array
    torque = sensor_reading.joint_torque_sensor[2]
    torque_array.append(torque)
    if torque > max_torque:
        rospy.set_param('Max_Torque', torque)

def emg_calib(hdEMG):
    global emg_array
    sample = np.mean(hdEMG.data)
    emg_array.append(sample) #pick specific value?
    
def main():
    rospy.init_node('calibration', log_level=rospy.DEBUG)
    h3_return = rospy.Subscriber('/h3/robot_states', State, torque_calib)
    #torque_sub = h3_return.joint_torque_sensor
    emg_sub = rospy.Subscriber('hdEMG_stream', Float64MultiArray, emg_calib)
    r = rospy.Rate(100)
    
    global torque_array
    global emg_array
    torque_array = []
    emg_array = []

    start = rospy.Time.now()
    duration = rospy.Duration.from_sec(trial_length) 
    while (rospy.Time.now() < start + duration):
        #logdebug(torque_array)
        #logdebug(emg_array)
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