#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import String, Float64, Float64MultiArray
from talker_listener.qc_predict import MUdecomposer
from rospy.core import logdebug

path = rospy.get_param("/file_dir")
#model_file = path + "\\src\\talker_listener\\best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model_file = path + "/src/talker_listener/best_model_cnn-allrun5_c8b_mix4-SG0-ST20-WS40-MU[0, 1, 2, 3]_1644222946_f.h5"
model = MUdecomposer(model_file)
sample = [] 
def predict_MUs(hdEMG):
    logdebug(hdEMG.data)
    global neural_drive
    if len(sample) < 124:
        sample.append(hdEMG.data)
    else:
        sample.pop()
        sample.append(hdEMG.data)
        neural_drive = model.predict_MUs(hdEMG.data)

    # neural_drive = random.randint(0,1) 

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
            # logdebug(torque_cmd)
        except:
            # logdebug("Could not find torque_cmd")
            continue

        pub.publish(torque_cmd)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass