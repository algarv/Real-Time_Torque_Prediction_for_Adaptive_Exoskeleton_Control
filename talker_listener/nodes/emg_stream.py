from tracemalloc import start
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from talker_listener.msg import hdemg
from rospy.core import logdebug 
import csv
import scipy as sp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal

def main():
    rospy.init_node('emg_stream', log_level=rospy.DEBUG)
    method = rospy.get_param('~method')
    print(method)
    r = rospy.Rate(512)
    pub = rospy.Publisher('hdEMG_stream', hdemg, queue_size=10)
    
    nyquist = .5 * 512.0 #2048.0
    filter_window = [20.0/nyquist, 500.0/nyquist]
    # avg_window = 100 #10**5

    filtering_window = 100
    high_b, high_a = signal.butter(4, 20/nyquist, btype='highpass')

    win = []

    data = []
    path = rospy.get_param("/file_dir")
    
    df = pd.read_csv(path+"/src/talker_listener/raw_emg_34.csv")
    df = df.iloc[:,1:].dropna()

    # stream = open(path + "/src/talker_listener/raw_emg_9_edit.csv")
    # csv_reader = csv.reader(stream, delimiter=',')
    
    # # csv_chunk = pd.read_csv(stream, chunksize=10000)
    # # for chunk in csv_chunk:
    # #     csv_reader = chunk.to_numpy()
    # for row in csv_reader:
    #     temp_list = []
    #     for num in row:
    #         temp_list.append(float(num))
    #     if len(temp_list) > 0:
    #         data.append(np.array(temp_list))
    
    # timer = rospy.get_time()

    data = df.to_numpy()
    print(data.shape)

    # b, a = signal.butter(4, filter_window, btype='bandpass')
    # filtered = signal.filtfilt(b, a, data, axis=0).tolist()

    sample_ready = False
    sample_count = 0
    i = 0
    while not rospy.is_shutdown():    
        # print("Measured Frequency: ", 1/(rospy.get_time() - timer))
        # timer = rospy.get_time()

    # for i in range(len(data)):
        stamped_sample = hdemg()
        stamped_sample.header.stamp = rospy.get_rostime()

        reading = data[i] #filtered[i]
        sample_count += 1
        i += 1
                    
        if i >= len(data): #len(row) - 4:
            i = 0

        if method == 'emg':
            if sample_count % 5 == 0:
                
                sample = Float64MultiArray()
                sample.data = reading

                dim = []
                dim.append(MultiArrayDimension("rows", 1, 6*64))
                dim.append(MultiArrayDimension("columns", 1, 1))

                sample.layout.dim = dim

                stamped_sample.data = sample


                pub.publish(stamped_sample)

        else:

            sample = Float64MultiArray()
            sample.data = reading

            dim = []
            dim.append(MultiArrayDimension("rows", 1, 6*64))
            dim.append(MultiArrayDimension("columns", 1, 1))

            sample.layout.dim = dim

            stamped_sample.data = sample


            pub.publish(stamped_sample)

        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass