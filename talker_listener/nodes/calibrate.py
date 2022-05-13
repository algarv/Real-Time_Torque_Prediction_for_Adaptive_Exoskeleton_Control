#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from State.msg import joint_motor_torque, joint_torque_sensor

sub = rospy.Subscriber('/h3/robot_states/joint_torque_sensor[5]', Float64MultiArray, calib)
torque_array = []
max_torque = 0

def calib(torque):
    torque_array.append(torque)
    if torque > max_torque:
        rospy.set_param('Max_Torque', torque)

def main():
    rospy.init_node('calibration', log_level=rospy.DEBUG)
    # calibrate = rospy.service('calibrate','calibration.srv',calib)
    return 0

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass