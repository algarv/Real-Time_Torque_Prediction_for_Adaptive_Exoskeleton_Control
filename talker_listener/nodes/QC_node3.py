#!/usr/bin/env python

import rospy
import random
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from talker_listener.qc_predict import MUdecomposer
from rospy.core import loginfo
from h3_msgs.msg import State

path = rospy.get_param("/file_dir")
#model_file = path + "\\src\\talker_listener\\best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model_file = path + "/src/talker_listener/" + "best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model = MUdecomposer(model_file)

win = 40
method = 'cst' 
adaptive = False #True
channels = [1,2,3,4]

mass = 2.37 #kg
g = -9.81 #m/s^2
l = 0.1524 #m

class QC_node:
    def __init__(self):
        r = rospy.Rate(100) #2048 HZ for reading samples, 33 HZ for predictions  #100Hz for torque controller
        self.dt = 1.0 / 100.0
        pub = rospy.Publisher('/h3/right_ankle_effort_controller/command', Float64, queue_size=10)
        emg_sub = rospy.Subscriber('hdEMG_stream', Float64MultiArray, self.get_sample)
        sensor_sub = rospy.Subscriber('/h3/robot_states', State, self.sensor_callback)
        self.first_run = True
        self.batch_ready = False
        n = len(channels)
        self.sample_count = 0
        self.sample = np.zeros((n, win, 64))

        self.torque_cmd = 0
        rospy.wait_for_message('/h3/robot_states', State,timeout=None)
        rospy.wait_for_message('hdEMG_stream', State,timeout=None)
        while not rospy.is_shutdown():
            if rospy.get_param('calibrated') == True:
                if self.first_run == True:
                    self.sample = np.zeros((n, win, 64))
                self.first_run = False

                # rospy.loginfo("Sample: ")
                # rospy.loginfo(self.sample)

                if method == 'cst':
                    self.torque_cmd = self.calc_torque_cst()
                
                if method == 'emg':
                    self.torque_cmd = self.calc_torque_emg()
            
                if adaptive:
                    self.torque_cmd = (self.torque_cmd - self.T_if) #+ (mass * g * l * np.sin(self.theta_next))

                rospy.loginfo("Torque_CMD: ")
                rospy.loginfo(self.torque_cmd)

                pub.publish(self.torque_cmd)
                r.sleep()

    def get_sample(self,hdEMG):
        
        num_groups = len(hdEMG.data) // 64

        samples = []
        for i in range(num_groups):
            muscle = list(hdEMG.data[64*i : 64*i + 64])
            samples.append(muscle)

        #n x 40 x 64 muscles multiarray
        i = 0
        for c in channels:
            if self.sample_count < win:
                self.sample[i][self.sample_count] = samples[c]
            else: # step size of 20 instead of 1
                deleted = np.delete(self.sample[i], 0, axis=0)
                self.sample[i] = np.append(deleted, [np.array(samples[c])],axis=0)
                self.batch_ready = True
            i += 1
        
        self.sample_count += 1
        

    def sensor_callback(self,sensor_reading):
        self.theta = sensor_reading.joint_position[2]
        self.theta_dot = sensor_reading.joint_velocity[2]
        self.net_torque = sensor_reading.joint_torque_sensor[2]


        self.T_if = self.net_torque - self.torque_cmd

        self.theta_next = self.theta + self.theta_dot * self.dt

    def calc_torque_cst(self):

        nueral_drive = model.predict_MUs(self.sample)
        print(nueral_drive)
        #Linear regression to map neural drive to torque

        return random.randint(-10,10)

    def calc_torque_emg(self):
        if self.batch_ready:
            m = rospy.get_param('slope')
            b = rospy.get_param('intercept')
            try: 
                torque_cmd = m * np.mean(self.sample) + b #need to take specific sample value
            except:
                rospy.loginfo("Waiting for sample")
                torque_cmd = 0
            
            return torque_cmd
        else:
            return 0 
        

if __name__ == '__main__':
    try:
        rospy.init_node('QC_node', log_level=rospy.DEBUG)
        node = QC_node()
    except rospy.ROSInterruptException:
        pass