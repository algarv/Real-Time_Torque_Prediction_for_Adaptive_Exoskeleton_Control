#!/usr/bin/env python
'''
QC_node_emg Node

Intermediate node to publish high density emg data from the Quattrocento 

Publishers:
    Name: 'hdEMG_stream' Type: talker_listener/hdemg
'''

import rospy
import socket
import time
import os
import numpy as np
import talker_listener.qc_communication as comm
import pandas as pd
import scipy as sp
from std_msgs.msg import String, Float64, Float64MultiArray, MultiArrayDimension
from talker_listener.msg import hdemg
from scipy import signal

def startup():
    # number of channels (408 for the quattrocento device)
    nchan = 384+16+8
    # sampling frequency (set in OT BioLab Light)
    fsamp = 512 #2048
    # number of bytes in sample (2 bytes for the quattrocento device)
    nbytes = 2
    # set duration of trial (seconds)
    nsec = 60
    # set buffer size (seconds)
    buffsize = 5

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
    
    return sample_from_channels

if __name__ == '__main__':
    rospy.init_node('QC_stream_node_512')
    r = rospy.Rate(512) # Match the Quattrocento publishing rate
    pub = rospy.Publisher('hdEMG_stream', hdemg, queue_size=1)
    
    q_socket, nchan, nbytes = startup()

    timer = rospy.get_time()
    win = []
    sample_count = 0

    while not rospy.is_shutdown():

        reading = record_print(q_socket, nchan, nbytes)
        stamped_sample = hdemg()
        stamped_sample.header.stamp = rospy.get_rostime()
        sample_count += 1

        sample = Float64MultiArray()
        sample.data = reading

        dim = []
        dim.append(MultiArrayDimension("rows", 1, 6*64))
        dim.append(MultiArrayDimension("columns", 1, 1))

        sample.layout.dim = dim

        stamped_sample.data = sample


        pub.publish(stamped_sample)

        r.sleep()