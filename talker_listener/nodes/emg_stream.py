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
    r = rospy.Rate(2048)
    pub = rospy.Publisher('hdEMG_stream', hdemg, queue_size=10)
    
    nyquist = .5 * 2048.0
    # filter_window = [20.0/nyquist, 50.0/nyquist]
    # avg_window = 100 #10**5

    filtering_window = 100
    high_b, high_a = signal.butter(4, 20/nyquist, btype='highpass')

    win = []

    data = []
    path = rospy.get_param("/file_dir")
    
    df = pd.read_csv(path+"/src/talker_listener/raw_emg_13.csv")
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
    
    data = df.to_numpy()
    print(data.shape)

    # b, a = signal.butter(4, filter_window, btype='bandpass')
    # filtered = signal.filtfilt(b, a, data, axis=0).tolist()

    sample_ready = False
    sample_count = 0
    i = 0
    while not rospy.is_shutdown():    
    # for i in range(len(data)):
        stamped_sample = hdemg()
        stamped_sample.header.stamp = rospy.get_rostime()

        reading = data[i] #filtered[i]
        sample_count += 1
        i += 1

        if sample_count <= filtering_window:
            win.append(reading)
            r.sleep()
        else:
            win.pop(0)
            win.append(reading)

            filtered=win.copy()
            for j in range(1,5):
                b, a = signal.iirnotch(123*j,30, 2048)
                filtered = signal.filtfilt(b,a, filtered, axis=0).tolist()
                filtered = np.array(filtered)

            b, a = signal.iirnotch(60,30, 2048)
            filtered = signal.filtfilt(b,a, filtered, axis=0).tolist()

            filtered = signal.filtfilt(high_b, high_a, filtered, axis=0).tolist()
            
            sample = Float64MultiArray()
            sample.data = filtered[-1] #smoothed_reading

            dim = []
            dim.append(MultiArrayDimension("rows", 1, 6*64))
            dim.append(MultiArrayDimension("columns", 1, 1))

            sample.layout.dim = dim

            stamped_sample.data = sample

            pub.publish(stamped_sample)
            
            if i >= len(data): #len(row) - 4:
                i = 0
            r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass