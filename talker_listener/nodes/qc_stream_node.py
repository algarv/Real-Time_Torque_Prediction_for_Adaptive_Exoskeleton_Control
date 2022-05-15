#!/usr/bin/env python

import rospy
import socket
import time
import os
import numpy as np
import talker_listener.qc_communication as comm
import pandas as pd
from std_msgs.msg import String, Float64, Float64MultiArray, MultiArrayDimension

# set save path
path = "C:\\Users\\Jackson\\Documents\\Jackson\\northwestern\\SRALAB\\H3" # "C:/Users/jlevine/Desktop"
# set file name
#datafile = "why_32768.csv"

def startup():
    # number of channels (408 for the quattrocento device)
    nchan = 384+16+8
    # sampling frequency (set in OT BioLab Light)
    fsamp = 2048
    # number of bytes in sample (2 bytes for the quattrocento device)
    nbytes = 2
    # set duration of trial (seconds)
    nsec = 60
    # set buffer size (seconds)
    buffsize = 5

    data = []
    timestamp = []

    start_comm = 'startTX'
    stop_comm = 'stopTX'

    ip_address = '127.0.0.1'
    port = 31000

    # Create a client socket which is used to connect to OT BioLab Light
    q_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    q_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, nchan*fsamp*buffsize)
    q_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    q_socket.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)

    # Establish client connection with server
    q_socket.connect((ip_address, port))
    print('waiting for connection...')

    # Start communication with the socket and receive 8 byte start message (OT BioLab)
    q_socket.send(start_comm.encode())
    time.sleep(0.01)
    msg = q_socket.recv(8)
    print(msg.decode("ascii") + " connected")
    q_socket.send(stop_comm.encode())
    
    return q_socket, nchan, nbytes

def record_print(q_socket, nchan, nbytes):
    start_comm = 'startTX'
    stop_comm = 'stopTX'

    q_socket.send(start_comm.encode())

    # Loop for receiving, converting and displaying incoming data
    time1 = time.time()
    # while True:
    # for i in range(fsamp*nsec):
    
    # read raw data from socket
    sbytes = comm.read_raw_bytes(
        q_socket,
        nchan,
        nbytes)

    # convert the bytes into integer values
    sample_from_channels = comm.bytes_to_integers(
        sbytes,
        nchan,
        nbytes,
        output_milli_volts=False)
    # print(sample_from_channels[384])
    # timestamp.append(time.time())
    #data.append(sample_from_channels)
    #print(sample_from_channels)
    return sample_from_channels
    #pub.publish(sample_from_channels)

    print(time.time()-time1)

    # End communication with the socket
    q_socket.send(stop_comm.encode())

    # Convert raw data and timestamps to array and save
    #dataset = np.array(data)
    # timestamps = np.array(timestamp)
    # timestamps = timestamps - timestamps[0]
    #df = pd.DataFrame(np.column_stack((timestamps, dataset)))

    channum = [str(n) for n in list(range(1, 65))]
    auxnum = [str(n) for n in list(range(1, 17))]
    othernum = [str(n) for n in list(range(1, 9))]

    inset = (['IN1-4']+['']*63)+(['IN5-8']+['']*63)
    multset = (['MULT IN1']+['']*63)+(['MULT IN2']+['']*63)+(['MULT IN3']+['']*63)+(['MULT IN4']+['']*63)
    auxset = (['AUX IN1-16']+['']*15)+(['OTHER']+['']*7)

    chansets = ['']+inset+multset+auxset
    numsets = ['time']+channum*6+auxnum+othernum

    df.columns = [chansets, numsets]
    return dataset #df.to_numpy()
    #df.to_csv(os.path.join(path, datafile), index=False)


if __name__ == '__main__':
    rospy.init_node('QC_stream_node')
    r = rospy.Rate(10) #10Hz
    pub = rospy.Publisher('hdEMG_stream', Float64MultiArray, queue_size=10)
    r = rospy.Rate(2048)
    
    q_socket, nchan, nbytes = startup()

    while not rospy.is_shutdown():
        sample = Float64MultiArray()
        sample.data = record_print(q_socket, nchan, nbytes)
        
        dim = []
        dim.append(MultiArrayDimension("rows",4,16*4))
        dim.append(MultiArrayDimension("columns",1,1))
        sample.layout.dim = dim        
        pub.publish(sample)
        r.sleep()