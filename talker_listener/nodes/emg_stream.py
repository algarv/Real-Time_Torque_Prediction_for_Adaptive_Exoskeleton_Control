import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
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
    pub = rospy.Publisher('hdEMG_stream', Float64MultiArray, queue_size=1)
    
    # nyquist = .5 * 2048.0
    # filter_window = [20.0/nyquist, 50.0/nyquist]
    # avg_window = 100 #10**5

    win = []

    data = []
    path = rospy.get_param("/file_dir")
    stream = open(path + "/src/talker_listener/raw_emg_4_edited.csv")
    csv_reader = csv.reader(stream, delimiter=',')
    
    # csv_chunk = pd.read_csv(stream, chunksize=10000)
    # for chunk in csv_chunk:
    #     csv_reader = chunk.to_numpy()
    for row in csv_reader:
        temp_list = []
        for num in row:
            temp_list.append(float(num))
        if len(temp_list) > 0:
            data.append(np.array(temp_list))
    
    data = np.array(data)
    print(data.shape)

    # b, a = signal.butter(4, filter_window, btype='bandpass')
    # filtered = signal.filtfilt(b, a, data, axis=0).tolist()

    sample_ready = False
    sample_count = 0
    i = 0
    while not rospy.is_shutdown():    
        reading = data[i] #filtered[i]
        #csv_reader = np.genfromtxt(stream,delimiter=',')
        #reading1 = []
        #reading2 = []
        #reading3 = []
        #reading4 = []
        #for row in csv_reader:
        #     reading1.append(float(row[i]))
        #     reading2.append(float(row[i+1]))
        #     reading3.append(float(row[i+2]))
        #     reading4.append(float(row[i+3]))
        # reading1.pop(0)
        # reading2.pop(0)
        # reading3.pop(0)
        # reading4.pop(0)

        #reading = reading1 + reading2 + reading3 + reading4

        # if sample_count < avg_window:
        #     win.append(reading)
        # else:
        #     win.pop(0)
        #     win.append(reading)
        #     sample_ready = True
        
        # if sample_ready:
        #     smoothed_reading = np.mean(win, axis=0)

        sample = Float64MultiArray()
        sample.data = reading #smoothed_reading

        dim = []
        dim.append(MultiArrayDimension("rows", 1, 6*64))
        dim.append(MultiArrayDimension("columns", 1, 1))

        sample.layout.dim = dim

        pub.publish(sample)
        
        i += 1 #4
        sample_count += 1
        if i >= len(data): #len(row) - 4:
            i = 0
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass