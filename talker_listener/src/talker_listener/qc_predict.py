#!python3

import numpy as np
import time
import talker_listener.qc_communication as comm
import tensorflow as tf
from talker_listener.hdEMG_DCNN import load_model_custom
from multiprocessing import Process, Queue
import socket
import os
import pandas as pd
import scipy.io as sio
#import matplotlib.pyplot as plt
import math


class MUdecomposer(object):
    def __init__(self, model_file=None):
        if model_file == None:
            raise ValueError("No model file specified")
        self.model_file = model_file
        # load model from h5 file
        self.model = load_model_custom(model_file)

    def predict_MUs(self, hdEMG):
        # predict and generate output
        self.preds = self.model.predict(hdEMG)
        self.preds_binary = tf.where(np.array(self.preds) >= 0.5, 1., 0.)
        return self.preds_binary


def plot_spikes(spike, munum):
    spikedata = np.array(spike, dtype=bool)
    spikeset = spikedata.reshape((spikedata.shape[1], spikedata.shape[0]))

    # Set different colors for each motor unit
    colorcodes = np.array([[0, 0, 0],
                           [1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1],
                           [1, 1, 0],
                           [1, 0, 1],
                           [0, 1, 1],
                           [0.5, 0.5, 0.5]])

    fig = plt.figure()
    fig.suptitle('Spiking Activity', fontsize=14)
    for mun in range(munum):
        # Draw a spike raster plot for each motor unit
        plt.eventplot(np.argwhere(spikeset[mun, :]).T, color=colorcodes[mun], linelengths=0.5, lineoffsets=mun+1)

    # Give y axis label for the spike raster plot
    plt.ylabel('MU number')
    # Display the spike raster plot
    plt.show()


def save_predict(spike, tHist, munum, path, predfile):
    # Convert prediction data and timestamps to array and save
    spikeset = np.array(spike)
    hist = np.array(tHist)
    spike_df = pd.DataFrame(np.column_stack((hist, spikeset.reshape(-1, munum))))
    spike_df.columns = [['tHist'] + [f'MU{j}' for j in range(1, munum + 1)]]
    spike_df.to_csv(os.path.join(path, predfile), index=False)


def save_emg(data, timestamp, setnum, path, datafile):
    # Convert raw data and timestamps to array and save
    dataset = np.array(data)
    timestamps = np.array(timestamp)
    timestamps = timestamps - timestamps[0]
    raw_df = pd.DataFrame(np.column_stack((timestamps, dataset)))

    channum = [str(n) for n in list(range(1, int(setnum/2 + 1)))]
    chanset = (['Raw GM'] + [''] * int(setnum/2 - 1)) + (['Raw TA'] + [''] * int(setnum/2 - 1))

    chansets = [''] + chanset
    numsets = ['time'] + channum + channum

    raw_df.columns = [chansets, numsets]
    raw_df.to_csv(os.path.join(path, datafile), index=False)


def save_force(force, timestamp, setnum, path, datafile):
    # Convert raw data and timestamps to array and save
    dataset = np.array(force)
    timestamps = np.array(timestamp)
    timestamps = timestamps - timestamps[0]
    raw_df = pd.DataFrame(np.column_stack((timestamps, dataset)))

    channum = [str(n) for n in list(range(1, setnum + 1))]
    chanset = (['Raw Force'] + [''] * (setnum - 1))

    chansets = [''] + chanset
    numsets = ['time'] + channum

    raw_df.columns = [chansets, numsets]
    raw_df.to_csv(os.path.join(path, datafile), index=False)


def emg_predict(emg_queue):
    # Loop for predicting spiking activity from incoming data
    spike = []
    tHist = []
    time1 = time.time()
    while True:
        if not emg_queue.empty():
            dat = emg_queue.get()
            if isinstance(dat, str):
                if dat == 'DONE':
                    save_predict(spike, tHist, munum, path, predfile)
                    plot_spikes(spike, munum)
                    print("emg_predict: " + str(time.time() - time1) + " seconds")
                    break
            else:
                start_time = time.time()
                if muscle == 'GM':
                    s = mude.predict_MUs(dat[:, :, :numchan])
                elif muscle == 'TA':
                    s = mude.predict_MUs(dat[:, :, numchan:])
                else:
                    s = [0, 0, 0, 0]

                tHist.append(time.time() - start_time)
                spike.append(s)
                print(s)
                return s


def emg_stream(queue, fsamp, nsec, q_socket, nchan, nbytes, chanset, setnum, stepsize, windowsize, matnorm, path, datafile, forcefile):
    data = []
    force = []
    timestamp = []
    print("streaming data...")
    time1 = time.time()
    # Loop for receiving, converting and segmenting incoming data
    for i in range(fsamp*nsec):

        # read raw data from socket
        sbytes = comm.read_raw_bytes(q_socket,
                                     nchan,
                                     nbytes)

        # convert the bytes into integer values
        sample_from_channels = comm.bytes_to_integers(sbytes,
                                                      nchan,
                                                      nbytes,
                                                      output_milli_volts=False)

        timestamp.append(time.time())
        data.append(sample_from_channels[chanset[0]:chanset[1]])
        force.append(sample_from_channels[384])

        if i % stepsize == 0 and i >= windowsize:
            queue.put(np.multiply(np.array(data[(i - windowsize + 1):]).reshape(1, windowsize, setnum), matnorm))

    queue.put('DONE')
    print("emg_stream: " + str(time.time()-time1) + " seconds")

    # End communication with the socket
    q_socket.send('stopTX'.encode())
    save_emg(data, timestamp, setnum, path, datafile)
    save_force(force, timestamp, 1, path, forcefile)
    print("You may disconnect from OTB Light now")


#######################################################################################################
#######################################################################################################

# number of channels (408 for the quattrocento device)
nchan = 384 + 16 + 8
# sampling frequency (set in OT BioLab Light)
fsamp = 2048
# number of bytes in sample (2 bytes for the quattrocento device)
nbytes = 2
# set duration of trial (seconds)
nsec = 60
# set buffer size (seconds)
buffsize = 5

# set save path
#path = "C:\\Users\\MSHORT\\PycharmProjects\\emgStreaming\\data\\pilot_20210524"  # "C:/Users/jlevine/Desktop"
# path = "C:\\opt\\ros\\noetic\\catkin2_ws\\src\\talker_listener"
# path = ""

# initialize trial parameters
trialnum = 3  # 1-3
intensity = 20  # 10%,20%,30%
angle = 0 # 0 or 20 degrees
muscle = 'GM'  # GM and TA
shape = 'ramp'  # ramp or sine

stepsize = 20
windowsize = 120
mu = [0, 1, 2, 3]

# range of channels to run prediction on
chanset = [128, 256]  # GM: [128, 192], TA: [192, 256], BOTH: [128, 256]
setnum = len(range(chanset[0], chanset[1]))
numchan = 64

# load model from h5 file
# modelFile = "technaid_h3_ankle_ros_python/compilation/best_model_cnn-0_0_DF.otb+_Tibialis anterior_Niter150_FastICA_JL-SG0-ST20-WS120-MU[0, 1, 2, 3]_1618602878_f.h5"
#modelFile = "best_model_cnn-0_0_PF.otb+_Gastrocnemius medialis_Niter150_FastICA_JL-SG0-ST20-WS120-MU[0, 1, 2, 3]_1618600237_f.h5"
#modelFile = "best_model_cnn-20_0_PF.otb+_Gastrocnemius medialis_Niter150_FastICA_JL-SG0-ST20-WS120-MU[0, 1, 2, 3]_1619757623_f.h5"

'''
# set file name
datafile = "{}_{}_{}_{}_{}-ST{}-WS{}-MU{}-EMG.csv".format(trialnum, intensity, angle, muscle, shape, stepsize, windowsize, mu)
predfile = "{}_{}_{}_{}_{}-ST{}-WS{}-MU{}-SPIKES.csv".format(trialnum, intensity, angle, muscle, shape, stepsize, windowsize, mu)
normfile = "{}_{}_{}_{}-ST{}-WS{}-MU{}-NORM.mat".format(intensity, angle, muscle, shape, stepsize, windowsize, mu)
forcefile = "{}_{}_{}_{}_{}-ST{}-WS{}-MU{}-FORCE.mat".format(trialnum, intensity, angle, muscle, shape, stepsize, windowsize, mu)

# EMG normalization from qc_norm
normdata = sio.loadmat(normfile)
matnorm = normdata['matnorm'][0]
matnorm = np.append(matnorm, matnorm)  # for both GM and TA
'''
# number of motor units (from model file)
# munum = len(mu)
# initialize motor unit decomposer
# mude = MUdecomposer(os.path.join(path,modelFile))

if __name__ == '__main__':

    # Create a client socket which is used to connect to OT BioLab Light
    ip_address = '127.0.0.1'
    port = 31000

    q_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    q_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, nchan * fsamp * buffsize)
    q_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    q_socket.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)

    # Establish client connection with server
    q_socket.connect((ip_address, port))
    print('waiting for connection...')

    # Start communication with the socket and receive 8 byte start message (OT BioLab)
    q_socket.send('startTX'.encode())
    time.sleep(0.01)
    msg = q_socket.recv(8)
    print(msg.decode("ascii") + " connected")

    # Initialize EMG queue
    emg_queue = Queue()  # emg_stream() writes to emg_queue from _this_ process

    ### emg_predict() reads from qc_queue as a separate process
    predict_emg = Process(target=emg_predict, args=(emg_queue, ))
    predict_emg.daemon = False
    predict_emg.start()  # Launch predict_emg() as a separate python process

    emg_stream(emg_queue, fsamp, nsec, q_socket, nchan, nbytes, chanset, setnum, stepsize, windowsize, matnorm, path, datafile, forcefile)
    predict_emg.join()  # wait for the reader to finish

    print("DONE")
