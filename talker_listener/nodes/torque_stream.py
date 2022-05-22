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

        if i < 5:
            reading[2] = i
        elif i < 10:
            reading[2] = 5
        else:
            reading[2] = 15 - i

        sample = State()
        sample.joint_torque_sensor = reading

        i += .01
        pub.publish(sample)
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass