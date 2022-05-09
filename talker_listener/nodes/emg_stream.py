import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from rospy.core import logdebug 
import csv




def main():
    rospy.init_node('QC_node', log_level=rospy.DEBUG)
    r = rospy.Rate(10) #100Hz for torque controller
    pub = rospy.Publisher('hdEMG_stream', Float64MultiArray, queue_size=10)
    
    i = 0
    while not rospy.is_shutdown():
        # reading = [0.37607, 1.4765, 1.67209, 1.723066, 1.49284, 1.4423, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        path = rospy.get_param("/file_dir")
        stream = open(path + "/src/talker_listener/offline_emg.csv")
        csv_reader = csv.reader(stream, delimiter=',')

        reading = []
        for row in csv_reader:
            reading.append(float(row[i]))
        reading.pop(0)
        logdebug(reading)

        sample = Float64MultiArray()
        sample.data = reading

        dim = []
        dim.append(MultiArrayDimension("rows", 4, 3*4))
        dim.append(MultiArrayDimension("columns", 4, 1))

        sample.layout.dim = dim

        i += 1
        pub.publish(sample)
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass