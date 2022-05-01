import rospy
from std_msgs.msg import String, Float64, Float64MultiArray, MultiArrayDimension
from rospy.core import logdebug 

def main():
    rospy.init_node('QC_node')
    r = rospy.Rate(100) #100Hz for torque controller
    pub = rospy.Publisher('hdEMG_stream', Float64MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        reading = [0.0,0.0,0.0,0.0, 0.01,0.01,0.01,0.01, 0.02,0.02,0.02,0.02, 0.03,0.03,0.03,0.03]
        sample = Float64MultiArray()
        sample.data = reading

        dim = []
        dim.append(MultiArrayDimension("rows", 4, 3*4))
        dim.append(MultiArrayDimension("columns", 4, 1))

        sample.layout.dim = dim

        pub.publish(sample)
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass