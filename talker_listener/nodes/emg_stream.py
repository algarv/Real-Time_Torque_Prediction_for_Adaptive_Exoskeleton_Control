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
        path = rospy.get_param("/file_dir")
        stream = open(path + "/src/talker_listener/offline_emg.csv")
        csv_reader = csv.reader(stream, delimiter=',')
        
        reading1 = []
        reading2 = []
        reading3 = []
        reading4 = []
        for row in csv_reader:
            reading1.append(float(row[i]))
            reading2.append(float(row[i+1]))
            reading3.append(float(row[i+2]))
            reading4.append(float(row[i+3]))
        reading1.pop(0)
        reading2.pop(0)
        reading3.pop(0)
        reading4.pop(0)

        reading = reading1 + reading2 + reading3 + reading4

        sample = Float64MultiArray()
        sample.data = reading

        dim = []
        dim.append(MultiArrayDimension("rows", 4, 16*4))
        dim.append(MultiArrayDimension("columns", 1, 1))

        sample.layout.dim = dim

        i += 4
        pub.publish(sample)
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass