#!/usr/bin/env python

import rospy 
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from sklearn import linear_model as lm
from rospy.core import logdebug

max_torque = 0
trial_length = 60 #seconds

def torque_calib(sensor_reading):
    global torque_array
    torque = sensor_reading.data[5]
    torque_array.append(torque)
    if torque > max_torque:
        rospy.set_param('Max_Torque', torque)

def emg_calib(hdEMG):
    global emg_array
    sample = np.mean(hdEMG.data)
    emg_array.append(sample) #pick specific value?
    
def main():
    rospy.init_node('calibration', log_level=rospy.DEBUG)
    torque_sub = rospy.Subscriber('/h3/robot_states/joint_torque_sensor', Float64MultiArray, torque_calib)
    emg_sub = rospy.Subscriber('hdEMG_stream', Float64MultiArray, emg_calib)
    r = rospy.Rate(1024)
    
    global torque_array
    global emg_array
    torque_array = []
    emg_array = []

    start = rospy.Time.now()
    duration = rospy.Duration.from_sec(trial_length) 
    while (rospy.Time.now() < start + duration):
        logdebug(torque_array)
        logdebug(emg_array)
        r.sleep()

    y = np.array(torque_array).reshape(-1,1)
    x = np.array(emg_array).reshape(-1,1)
    model = lm.LinearRegression()
    res=model.fit(x,y)
    intercept = res.params[0]
    slope = res.params[1]
    rospy.set_param('Intercept',intercept)
    rospy.set_param('Slope',slope)
    rospy.set_param('calibrated', True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass