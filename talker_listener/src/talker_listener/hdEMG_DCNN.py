# from sklearn.decomposition import PCA
# from sklearn.preprocessing import normalize
#import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
from os import path
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle


# load data from mat files
# including two variables: EMGs and spikes
def load_data_mat(TR, SG=0, ST=10, MU=1, WS=120, TF=0, MutiSeg=0):
    # TR - trial name (e.g., 1_30_GM)
    # SG - segment ID (e.g., 0-2)
    # ST - step size (5, 10, 20, 30, 40, 50)
    # MU - motor unit index (0-N, N is the number)
    # WS - window size (e.g., 120)
    # TF = 0, no shuffle; TF = 1, shuffle;  0<TF<1, seperate data
    # MutiSeg - 0: train with one segment of data; 1: train with two segments of data

    seg = [1, 2, 3]
    # load train data set
    segment = seg[SG]
    # construct mat file name based on parameters
    prefix = "{}-SG{}-WS{}-ST{}".format(TR, segment, WS, ST)
    matfile = "{}.mat".format(prefix)
    if not path.exists(matfile):
        pathstr = 'D:\\emg_data\\'  # data folder for emg
        matfile = "{}{}".format(pathstr, matfile)

    vnames = ['EMGs', 'Spikes']
    # load mat file
    data = sio.loadmat(matfile, variable_names=vnames)
    x_data = data['EMGs']
    spikes = data['Spikes']

    # load second segment if MutiSeg is 1
    if MutiSeg:
        seg2 = [2, 3, 1]
        segment = seg2[SG]
        prefix = "{}-SG{}-WS{}-ST{}".format(TR, segment, WS, ST)
        matfile = "{}.mat".format(prefix);
        if not path.exists(matfile):
            pathstr = 'D:\\emg_data\\'
            matfile = "{}{}".format(pathstr, matfile)
        #     print(matfile)
        data_2 = sio.loadmat(matfile, variable_names=vnames)
        x_data_2 = data_2['EMGs']
        spikes_2 = data_2['Spikes']
        x_data = np.concatenate((x_data, x_data_2))
        spikes = np.concatenate((spikes, spikes_2))

    #     x_data.shape
    # exactract spikes for given motor units
    if type(MU) is list:
        y_data = []
        for c in MU:
            if c < spikes.shape[1]:
                y_data.append(spikes[:, c])
            else:
                y_data.append(spikes[:, -1] * 0)
    else:
        y_data = []
        y_data.append(spikes[:, MU])

    ## shuffle the data based on TF flag
    y_data = np.array(y_data)
    y_data = y_data.T
    if TF == 1:
        x_data, y_data = shuffle(x_data, y_data)
    elif TF > 0:
        x_data, _, y_data, _ = train_test_split(x_data, y_data, test_size=1.0 - TF)
    else:
        print('no shuffle')
    y_data = y_data.T
    y_data = list(y_data)

    return x_data, y_data


import tensorflow.keras.backend as K
from tensorflow.keras.metrics import Metric
from tensorflow.keras.callbacks import Callback
import tensorflow as tf


# import neptune

# calculate f1 score
def f1_m(y_true, y_pred):
    #     precision = precision_m(y_true, y_pred)
    #     recall = recall_m(y_true, y_pred)
    y_pred_binary = tf.where(y_pred >= 0.5, 1., 0.)
    true_positives = K.sum(K.round(K.clip(y_true * y_pred_binary, 0, 1)))
    possible_positives = K.sum(K.round(K.clip(y_true, 0, 1)))
    predicted_positives = K.sum(K.round(K.clip(y_pred_binary, 0, 1)))

    precision = true_positives / (predicted_positives + K.epsilon())
    recall = true_positives / (possible_positives + K.epsilon())
    return 2 * ((precision * recall) / (precision + recall + K.epsilon()))


# customized callback function to calculate averaged f1_score and accuracy across all outputs
class AccuracyCallback(Callback):
    def __init__(self, metric_name='accuracy'):
        super().__init__()
        self.metric_name = metric_name
        self.val_metric = []
        self.metric = []
        self.val_metric_mean = 0
        self.metric_mean = 0
        self.best_metric = 0

    def on_epoch_end(self, epoch, logs=None):
        #         print('Accuracycallback')
        # extract values from logs
        self.val_metric = []
        self.metric = []
        for log_name, log_value in logs.items():
            if log_name.find(self.metric_name) != -1:
                if log_name.find('val') != -1:
                    self.val_metric.append(log_value)
                else:
                    self.metric.append(log_value)

        self.val_metric_mean = np.mean(self.val_metric)
        self.metric_mean = np.mean(self.metric)
        logs['val_{}'.format(self.metric_name)] = np.mean(self.val_metric)  # replace it with your metrics
        logs['{}'.format(self.metric_name)] = np.mean(self.metric)  # replace it with your metrics


import tensorflow.keras as keras
# import keras
from tensorflow.keras.preprocessing.sequence import TimeseriesGenerator
from tensorflow.keras.models import Sequential, load_model, Model
from tensorflow.keras.layers import Dense, Flatten, Activation, Input
from tensorflow.keras.layers import Conv1D, MaxPooling1D, Conv2D, MaxPooling2D, BatchNormalization, Dropout, LSTM
from tensorflow.keras.regularizers import l2


# create convolutional neural network with sequential model
def get_cnn1d_model(shape_in, shape_out, nn_nodes=[128, 128, 128, 64, 256]):
    '''Create a keras model.'''
    # shape_in = (timesteps, features)
    model = Sequential()
    gg_nn_nodes = nn_nodes
    print(gg_nn_nodes)

    # add model layers, number of filter, kernel_size
    model.add(Conv1D(filters=gg_nn_nodes[0], kernel_size=3, activation='relu', input_shape=shape_in))
    model.add(Conv1D(filters=gg_nn_nodes[1], kernel_size=3, activation='relu'))
    model.add(MaxPooling1D(pool_size=2))
    model.add(Dropout(0.5))

    model.add(Conv1D(filters=gg_nn_nodes[2], kernel_size=3, activation='relu'))
    model.add(Conv1D(filters=gg_nn_nodes[3], kernel_size=3, activation='relu'))
    model.add(MaxPooling1D(pool_size=2))
    model.add(Dropout(0.5))

    model.add(Flatten())
    model.add(Dense(gg_nn_nodes[4], activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(shape_out, activation='sigmoid'))

    return model


############ create models with given input shape and output shape ##########
# create convolutional neural network with API interface
def get_cnn1d_api(shape_in, shape_out, nn_nodes=[128, 128, 128, 64, 256]):
    '''Create a keras model with functional API'''
    # create convolutional neural network model
    # shape_in = (timesteps, features)
    print(nn_nodes)

    # create shared layers
    visible = Input(shape=shape_in, name='EMG')
    cnn = Conv1D(filters=nn_nodes[0], kernel_size=3, activation='relu')(visible)
    cnn = Conv1D(filters=nn_nodes[1], kernel_size=3, activation='relu')(cnn)
    cnn = MaxPooling1D(pool_size=2)(cnn)
    cnn = Dropout(0.5)(cnn)

    # create seperate layers for each motor unit
    outputs = []
    for k in range(1, shape_out + 1):
        cnn_2 = Conv1D(filters=nn_nodes[2], kernel_size=3, activation='relu')(cnn)
        cnn_2 = Conv1D(filters=nn_nodes[3], kernel_size=3, activation='relu')(cnn_2)
        cnn_2 = MaxPooling1D(pool_size=2)(cnn_2)
        cnn_2 = Dropout(0.5)(cnn_2)

        cnn_2 = Flatten()(cnn_2)
        s2 = Dense(nn_nodes[4], activation='relu')(cnn_2)
        s2 = Dropout(0.5)(s2)
        output = Dense(1, activation='sigmoid', name='output_{}'.format(k))(s2)
        outputs.append(output)

    # construct metrics and loss configuration
    metrics = {'output_1': ['accuracy', f1_m]}
    loss = {'output_1': 'binary_crossentropy'}
    for k in range(2, shape_out + 1):
        key = 'output_{}'.format(k)
        metrics[key] = ['accuracy', f1_m]
        loss[key] = 'binary_crossentropy'

    # tie together
    model = Model(inputs=visible, outputs=outputs)
    return model, loss, metrics


# use tensorboard for display
from tensorflow.keras.callbacks import TensorBoard, EarlyStopping, ModelCheckpoint, LambdaCallback
import time
#import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
from tensorflow.keras.models import load_model
from sklearn.model_selection import train_test_split


# build model with configuration
def build_model(WS=120, n_output=1, nn_nodes=[128, 128, 128, 64, 256]):
    # mIndex is not in use
    n_input = WS
    n_features = 64  # set default number of EMG channels

    if n_output == 1:
        print('Sequential model')
        model_cnn = get_cnn1d_model((n_input, n_features), n_output, nn_nodes)
        loss_cnn = 'binary_crossentropy'
        metrics_cnn = [
            'accuracy',
            'mse',
            f1_m,
        ]
    else:
        print('API model')
        model_cnn, loss_cnn, metrics_cnn = get_cnn1d_api((n_input, n_features), n_output, nn_nodes)

    model = model_cnn
    model.compile(optimizer='rmsprop',  # sgd', 'adagrad', 'rmsprop', 'adam'
                  loss=loss_cnn,  # mean_squared_error
                  metrics=metrics_cnn)  # ['accuracy', 'mse'])
    return model_cnn


def train_model(model, x_data, y_data, prefix, epochs=100):
    tname = int(time.time())
    batch_size = 64

    # create tersorboard
    log_name = "hdEMG_{}_{}".format(prefix, tname)
    tensorboard = TensorBoard(log_dir=".\\logs\\{}".format(log_name))
    # check tenorboard by running
    # tensorboard --logdir C:\Users\Yue\Documents\mudecomp\logs\ # in the anaconda prompt command line
    # tensorboard --logdir C:\Users\ywen.SMPP\Documents\mudecomp\logs\

    # early stop when loss improvement is small
    es = EarlyStopping(monitor='loss', mode='min', verbose=1, patience=50)

    # save the best model when loss is minimum and f1_score is highest
    mc = ModelCheckpoint('best_model_{}_{}_l.h5'.format(prefix, tname), monitor='loss', mode='min', verbose=1,
                         save_best_only=True)
    mc_vl = ModelCheckpoint('best_model_{}_{}_vl.h5'.format(prefix, tname), monitor='val_loss', mode='min', verbose=1,
                            save_best_only=True)
    mc_f = ModelCheckpoint('best_model_{}_{}_f.h5'.format(prefix, tname), monitor='f1_m', mode='max', verbose=1,
                           save_best_only=True)
    mc_vf = ModelCheckpoint('best_model_{}_{}_vf.h5'.format(prefix, tname), monitor='val_f1_m', mode='max', verbose=1,
                            save_best_only=True)

    # create customized callbacks
    accuracy_callback = AccuracyCallback('accuracy')
    f1_callback = AccuracyCallback('f1_m')

    # train model
    history = model.fit(x_data,
                        y_data,
                        validation_split=0.2,
                        batch_size=batch_size,
                        epochs=epochs,
                        verbose=1,
                        callbacks=[es, mc, mc_vl, accuracy_callback, f1_callback, tensorboard, mc_f, mc_vf])

    # return best model for further evaluation
    model = load_model(model_name, custom_objects={"f1_m": f1_m})
    return model, tname


# display model structure
# pip install pydotplus
# pip install pydot
# https://bobswift.atlassian.net/wiki/spaces/GVIZ/pages/20971549/How+to+install+Graphviz+software
def display_model(model, filename='model.png'):
    # plot model structure
    from tensorflow.keras.utils import plot_model
    plot_model(model, to_file='C:\{}'.format(filename), show_shapes=True)
    from IPython.display import Image
    Image(filename='C:\{}'.format(filename))


# load model with cuostmized metrics
def load_model_custom(model_name):
    model = load_model(model_name, custom_objects={"f1_m": f1_m})
    return model


# validate model with given data sets
def model_validate(model, x_data, y_data, prefix):
    # sequential data
    y_pred = evaluate(model, x_data, y_data)
    savedata(y_data, y_pred, "{}".format(prefix))


# evaluate model prediction
def evaluate(model, x_val, y_val, showFlag=0):
    #
    print('\n# Generate predictions')
    y_pred = model.predict(x_val)
    y_pred = np.asarray(y_pred)
    if len(y_pred.shape) == 3:
        y_pred = np.reshape(y_pred, (y_pred.shape[0], y_pred.shape[1]))

    if showFlag:
        f, (ax1, ax2) = plt.subplots(1, 2)
        ax1.plot(y_val)
        ax1.set_title('real_value')
        ax2.plot(y_pred)
        ax2.set_title('predict_value')
        plt.show
    return y_pred


# save prediction and actual values to csv file
def savedata(y_val, y_pred, fname):
    # convert to array if y_val type is list
    if type(y_val) is list:
        y_val = np.array(y_val)
    if type(y_pred) is list:
        y_pred = np.array(y_pred)

    # reshape
    if len(y_pred.shape) == 3:
        y_pred = np.reshape(y_pred, (y_pred.shape[0], y_pred.shape[1]))
    if len(y_val.shape) == 3:
        y_val = np.reshape(y_val, (y_val.shape[0], y_val.shape[1]))

    # rotate
    if len(y_val.shape) == 2 and y_val.shape[0] < y_val.shape[1]:
        y_val = np.transpose(y_val)
    elif len(y_val.shape) == 1:
        y_val = np.reshape(y_val, (y_val.shape[0], 1))

    if y_pred.shape[0] < y_pred.shape[1]:
        y_val = np.transpose(y_val)

    # save data
    if y_val.shape[0] > y_val.shape[1] and y_val.shape[0] == y_pred.shape[0]:
        data = np.column_stack((y_val, y_pred))
    #         data = np.transpose(data)
    else:
        data = np.vstack((y_val, y_pred))

    if data.shape[0] < data.shape[1]:
        data = np.transpose(data)
    data.shape
    pd.DataFrame(data).to_csv("output-{}.csv".format(fname))