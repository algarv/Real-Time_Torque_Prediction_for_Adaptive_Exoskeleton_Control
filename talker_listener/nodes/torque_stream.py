import rospy
import random
from h3_msgs.msg import State
from rospy.core import logdebug 

def main():
    rospy.init_node('torque_stream', log_level=rospy.DEBUG)
    r = rospy.Rate(100)
    pub = rospy.Publisher('/h3/robot_states', State, queue_size=10)

    reading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    i = 0
    while not rospy.is_shutdown():

        reading[2] = 2
        # if i < 3:
        #     reading[2] = 0
        # elif i < 8: 
        #     reading[2] = i
        # elif i < 18:
        #     reading[2] = 1
        # elif i < 23:
        #     reading[2] = 3 - i
        # else: 
        #     reading[2] = 0

        sample = State()
        sample.joint_torque_sensor = reading

        i += .01
        pub.publish(sample)
        if i > 26:
            i = 0
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass