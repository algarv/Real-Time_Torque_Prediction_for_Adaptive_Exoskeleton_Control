#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import String, Float64, Float64MultiArray
# from talker_listener.qc_predict import MUdecomposer
from rospy.core import logdebug

# model_file = "technaid_h3_ankle_ros_python/talker_listener/src/talker_listener/best_model_cnn-0_0_DF.otb+_Tibialis anterior_Niter150_FastICA_JL-SG0-ST20-WS120-MU[0, 1, 2, 3]_1618602878_f.h5"

# model = MUdecomposer(model_file)

def predict_MUs(hdEMG):
    global neural_drive
    # neural_drive = model.predict_MUs(hdEMG)
    neural_drive = random.randint(0,1) 

def calc_torque(neural_drive):
    #Linear regression to map neural drive to torque
    return random.randint(-10,10)

def main():
    rospy.init_node('QC_node', log_level=rospy.DEBUG)
    r = rospy.Rate(100) #100Hz for torque controller

    pub = rospy.Publisher('/h3/right_ankle_effort_controller/command', Float64, queue_size=10)
    sub = rospy.Subscriber('hdEMG_stream', Float64MultiArray, predict_MUs)

    while not rospy.is_shutdown():

        global neural_drive
        try:
            torque_cmd = calc_torque(neural_drive)
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