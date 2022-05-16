#!/usr/bin/env python

import rospy
import random
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from talker_listener.qc_predict import MUdecomposer
from rospy.core import logdebug

path = rospy.get_param("/file_dir")
#model_file = path + "\\src\\talker_listener\\best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model_file = path + "/src/talker_listener/" + "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model = MUdecomposer(model_file)

win = 40
method = 'emg' # 'cst'

def get_sample(hdEMG):
    global sample 
    
    if len(sample) < win:
        sample.append(hdEMG.data)
    else:
        sample.pop()
        sample.append(hdEMG.data)

def calc_torque_cst():
    global sample

    nueral_drive = model.predict_MUs(sample)
    #Linear regression to map neural drive to torque

    return random.randint(-10,10)

def calc_torque_emg():
    global sample
    m = rospy.get_param('slope')
    b = rospy.get_param('intercept')
    try: 
        torque_cmd = m * sample + b #need to take specific sample value
    except:
        logdebug("Waiting for sample")
        torque_cmd = 0
    
    return torque_cmd

def main():
    rospy.init_node('QC_node', log_level=rospy.DEBUG)
    r = rospy.Rate(10) #100Hz for torque controller
    pub = rospy.Publisher('/h3/right_ankle_effort_controller/command', Float64, queue_size=10)
    sub = rospy.Subscriber('hdEMG_stream', Float64MultiArray, get_sample)
    first_run = True
    while not rospy.is_shutdown():
        global sample
        sample = []
        if rospy.get_param('calibrated') == True:
            if first_run == True:
                sample = []
            first_run = False
            try:
                if method == 'cst':
                    torque_cmd = calc_torque_cst()
                
                if method == 'emg':
                    torque_cmd = 10*calc_torque_emg()
            
                logdebug("Torque_CMD: ")
                logdebug(torque_cmd)
            except:
                logdebug("Could not find torque_cmd")
                continue

            pub.publish(torque_cmd)
            r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass