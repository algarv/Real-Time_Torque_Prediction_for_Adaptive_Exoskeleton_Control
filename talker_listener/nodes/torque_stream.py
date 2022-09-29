import rospy
import random
from h3_msgs.msg import State
from rospy.core import logdebug 

def main():
    rospy.init_node('torque_stream', log_level=rospy.DEBUG)
    r = rospy.Rate(100)
    pub = rospy.Publisher('/h3/robot_states', State, queue_size=10)

    reading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    step =  .20 * (2.0) / 5
    i = 0
    while not rospy.is_shutdown():

        
        reading[2] = 2
        # if i < 3:
        #     reading[2] = 0
        # elif i < 8: 
        #     reading[2] = i * step
        # elif i < 18:
        #     reading[2] = .5
        # elif i < 23:
        #     reading[2] = .5 - (i*step)
        # else: 
        #     reading[2] = 0

        sample = State()
        sample.joint_torque_sensor = reading
        sample.joint_position = reading
        sample.joint_velocity = reading
        sample.joint_motor_torque = reading
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