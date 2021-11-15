#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Float64, Int32MultiArray
from talker_listener.test_script import Mult2
from talker_listener.qc_stream import record_print


def talker():
    pub = rospy.Publisher('/h3/right_ankle_effort_controller/command', Float64, queue_size=10)
    #pub_qc = rospy.Publisher('quattrocento', Float64, queue_size=10)
    rospy.init_node('talker')
    rate = rospy.Rate(10)  # 10hz
    t = 0
    x = Mult2()
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        out = x.mult2(t)
        if out>5:
            t = -5
        rospy.loginfo(out)
        #pub.publish(out)
        emg = record_print()
        rospy.loginfo(emg*3)
        pub.publish(emg*3)
        rate.sleep()
        t+=1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
