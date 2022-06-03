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
method = 'cst' #'emg'

class QC_node:
    def __init__(self):
        r = rospy.Rate(10) #100Hz for torque controller
        pub = rospy.Publisher('/h3/right_ankle_effort_controller/command', Float64, queue_size=10)
        sub = rospy.Subscriber('hdEMG_stream', Float64MultiArray, self.get_sample)
        self.first_run = True
        self.sample = []
        while not rospy.is_shutdown():
            if rospy.get_param('calibrated') == True:
                if self.first_run == True:
                    self.sample = []
                self.first_run = False

                # logdebug("Sample: ")
                # logdebug(self.sample)

                if method == 'cst':
                    torque_cmd = self.calc_torque_cst()
                
                if method == 'emg':
                    torque_cmd = self.calc_torque_emg()
            
                logdebug("Torque_CMD: ")
                logdebug(torque_cmd)

                pub.publish(torque_cmd)
                r.sleep()

    def get_sample(self,hdEMG):
        
        if len(self.sample) < win:
            self.sample.append(hdEMG.data)
        else:
            self.sample.pop()
            self.sample.append(hdEMG.data)
        
    def calc_torque_cst(self):

        nueral_drive = model.predict_MUs(self.sample)
        #Linear regression to map neural drive to torque

        return random.randint(-10,10)

    def calc_torque_emg(self):
        m = rospy.get_param('slope')
        b = rospy.get_param('intercept')
        try: 
            torque_cmd = m * np.mean(self.sample) + b #need to take specific sample value
        except:
            logdebug("Waiting for sample")
            torque_cmd = 0
        
        return torque_cmd
        

if __name__ == '__main__':
    try:
        rospy.init_node('QC_node', log_level=rospy.DEBUG)
        node = QC_node()
    except rospy.ROSInterruptException:
        pass