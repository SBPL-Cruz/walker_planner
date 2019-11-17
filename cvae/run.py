import torch
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
import os
import csv
from random import randint, random
import time
import argparse

from model import CVAE

def parse_arguments():
    # Command-line flags are defined here.
    parser = argparse.ArgumentParser()
    parser.add_argument('--run_id', dest='run_id', type=str, default=1)
    parser.add_argument('--exp_path_prefix', dest='experiment_path_prefix', type=str)
    return parser.parse_args()

def train(run_id=1):
    model = CVAE(run_id=run_id)

if __name__ == "__main__":
    args = parse_arguments()
    run_id = args.run_id
    train(run_id=run_id)


# (restrict tensorflow memory growth)
# os.environ["CUDA_VISIBLE_DEVICES"]="1"
# config = tf.ConfigProto()
# config.gpu_options.allow_growth=True

# # neural network parameters
# mb_size = 256
# h_Q_dim = 512
# h_P_dim = 512

# c = 0
# lr = 1e-4

# # problem dimensions
# dim = 6
# dataElements = dim+3*3+2*dim # sample (6D), gap1 (2D, 1D orientation), gap2, gap3, init (6D), goal (6D)

# z_dim = 3 # latent
# X_dim = dim # samples
# y_dim = dim # reconstruction of the original point
# c_dim = dataElements - dim # dimension of conditioning variable

# # read in data from csv
# filename = 'narrowDataFile.txt'
# f = open(filename, 'rb')
# reader = csv.reader(f, delimiter=',')
# count = 0
# data_list = []
# for row in reader:
#     data_list.append(map(float,row[0:dataElements]))

# data = np.array(data_list,dtype='d')
# numEntries = data.shape[0]


# # split the inputs and conditions into test train (to be processed in the next step into an occupancy grid representation)
# ratioTestTrain = 0.8;
# numTrain = int(numEntries*ratioTestTrain)

# X_train = data[0:numTrain,0:dim] # state: x, y, z, xdot, ydot, zdot
# c_train = data[0:numTrain,dim:dataElements] # conditions: gaps, init (6), goal (6)

# X_test = data[numTrain:numEntries,0:dim]
# c_test = data[numTrain:numEntries,dim:dataElements]
# numTest = X_test.shape[0]

#  change conditions to occupancy grid
# def isSampleFree(sample, obs):
#     for o in range(0,obs.shape[0]/(2*dimW)):
#         isFree = 0
#         for d in range(0,sample.shape[0]):
#             if (sample[d] < obs[2*dimW*o + d] or sample[d] > obs[2*dimW*o + d + dimW]):
#                 isFree = 1
#                 break
#         if isFree == 0:
#             return 0
#     return 1

# gridSize = 11
# dimW = 3
# plotOn = False;

# # process data into occupancy grid
# conditions = data[0:numEntries,dim:dataElements]
# conditionsOcc = np.zeros([numEntries,gridSize*gridSize])
# occGridSamples = np.zeros([gridSize*gridSize, 2])
# gridPointsRange = np.linspace(0,1,num=gridSize)

# idx = 0;
# for i in gridPointsRange:
#     for j in gridPointsRange:
#         occGridSamples[idx,0] = i
#         occGridSamples[idx,1] = j
#         idx += 1;

# start = time.time();
# for j in range(0,numEntries,1):
#     dw = 0.1
#     dimW = 3
#     gap1 = conditions[j,0:3]
#     gap2 = conditions[j,3:6]
#     gap3 = conditions[j,6:9]
#     init = conditions[j,9:15]
#     goal = conditions[j,15:21]

#     obs1 = [0, gap1[1]-dw, -0.5,             gap1[0], gap1[1], 1.5]
#     obs2 = [gap2[0]-dw, 0, -0.5,             gap2[0], gap2[1], 1.5];
#     obs3 = [gap2[0]-dw, gap2[1]+dw, -0.5,    gap2[0], 1, 1.5];
#     obs4 = [gap1[0]+dw, gap1[1]-dw, -0.5,    gap3[0], gap1[1], 1.5];
#     obs5 = [gap3[0]+dw, gap1[1]-dw, -0.5,    1, gap1[1], 1.5];
#     obs = np.concatenate((obs1, obs2, obs3, obs4, obs5), axis=0)
    
#     if j % 5000 == 0:
#         print('Iter: {}'.format(j))
        
#     occGrid = np.zeros(gridSize*gridSize)
#     for i in range(0,gridSize*gridSize):
#         occGrid[i] = isSampleFree(occGridSamples[i,:],obs)
#     conditionsOcc[j,:] = occGrid
    
#     if plotOn:
#         fig1 = plt.figure(figsize=(10,6), dpi=80)
#         ax1 = fig1.add_subplot(111, aspect='equal')
#         for i in range(0,obs.shape[0]/(2*dimW)): # plot obstacle patches
#             ax1.add_patch(
#             patches.Rectangle(
#                 (obs[i*2*dimW], obs[i*2*dimW+1]),   # (x,y)
#                 obs[i*2*dimW+dimW] - obs[i*2*dimW],          # width
#                 obs[i*2*dimW+dimW+1] - obs[i*2*dimW+1],          # height
#                 alpha=0.6
#             ))
#         for i in range(0,gridSize*gridSize): # plot occupancy grid
#             if occGrid[i] == 0:
#                 plt.scatter(occGridSamples[i,0], occGridSamples[i,1], color="red", s=70, alpha=0.8)
#             else:
#                 plt.scatter(occGridSamples[i,0], occGridSamples[i,1], color="green", s=70, alpha=0.8)
#         plt.show()
# end = time.time();
# print('Time: ', end-start)
    
# cs = np.concatenate((data[0:numEntries,dim+3*dimW:dataElements], conditionsOcc), axis=1) # occ, init, goal
# c_dim = cs.shape[1]
# c_gapsInitGoal = c_test
# c_train = cs[0:numTrain,:] 
# c_test = cs[numTrain:numEntries,:]


#  define networks
# X = tf.placeholder(tf.float32, shape=[None, X_dim])
# c = tf.placeholder(tf.float32, shape=[None, c_dim])

# # Q
# inputs_Q = tf.concat(axis=1, values=[X,c])

# dense_Q1 = tf.layers.dense(inputs=inputs_Q, units=h_Q_dim, activation=tf.nn.relu)
# dropout_Q1 = tf.layers.dropout(inputs=dense_Q1, rate=0.5)
# dense_Q2 = tf.layers.dense(inputs=dropout_Q1, units=h_Q_dim, activation=tf.nn.relu)

# z_mu = tf.layers.dense(inputs=dense_Q2, units=z_dim) # output here is z_mu
# z_logvar = tf.layers.dense(inputs=dense_Q2, units=z_dim) # output here is z_logvar

# # P
# eps = tf.random_normal(shape=tf.shape(z_mu))
# z = z_mu + tf.exp(z_logvar / 2) * eps
# inputs_P = tf.concat(axis=1, values=[z,c])

# dense_P1 = tf.layers.dense(inputs=inputs_P, units=h_P_dim, activation=tf.nn.relu)
# dropout_P1 = tf.layers.dropout(inputs=dense_P1, rate=0.5)
# dense_P2 = tf.layers.dense(inputs=dropout_P1, units=h_P_dim, activation=tf.nn.relu)

# y = tf.layers.dense(inputs=dense_P2, units=X_dim) # fix to also output y

# # training
# w = [[1, 1, 1, 0.5, 0.5, 0.5]];
# recon_loss = tf.losses.mean_squared_error(labels=X, predictions=y, weights=w)
# # TODO: fix loss function for angles going around
# kl_loss = 10**-4 * 2 * tf.reduce_sum(tf.exp(z_logvar) + z_mu**2 - 1. - z_logvar, 1)

# cvae_loss = tf.reduce_mean(kl_loss + recon_loss)

# train_step = tf.train.AdamOptimizer(lr).minimize(cvae_loss)

# sess = tf.Session(config=config)
# sess.run(tf.global_variables_initializer())
# it = 0;

# for it in range(it,it+500001):
#     # randomly generate batches
#     batch_elements = [randint(0,numTrain-1) for n in range(0,mb_size)]
#     X_mb = X_train[batch_elements,:]
#     c_mb = c_train[batch_elements,:]

#     _, loss = sess.run([train_step, cvae_loss], feed_dict={X: X_mb, c: c_mb})

#     if it % 1000 == 0:
#         print('Iter: {}'.format(it))
#         print('Loss: {:.4}'. format(loss))
#         print()
        
# # plot the latent space
# num_viz = 3000

# vizIdx = randint(0,numTest-1);
# print vizIdx
# c_sample_seed = c_test[vizIdx,:]
# c_sample = np.repeat([c_sample_seed],num_viz,axis=0)
# c_viz = c_gapsInitGoal[vizIdx,:]

# # directly sample from the latent space (preferred, what we will use in the end)
# y_viz, z_viz = sess.run([y, z], feed_dict={z: np.random.randn(num_viz, z_dim), c: c_sample})

# fig1 = plt.figure(figsize=(10,6), dpi=80)
# ax1 = fig1.add_subplot(111, aspect='equal')

# plt.scatter(y_viz[:,0],y_viz[:,1], color="green", s=70, alpha=0.1)

# dw = 0.1
# dimW = 3
# gap1 = c_viz[0:3]
# gap2 = c_viz[3:6]
# gap3 = c_viz[6:9]
# init = c_viz[9:15]
# goal = c_viz[15:21]

# obs1 = [0, gap1[1]-dw, -0.5,             gap1[0], gap1[1], 1.5]
# obs2 = [gap2[0]-dw, 0, -0.5,             gap2[0], gap2[1], 1.5];
# obs3 = [gap2[0]-dw, gap2[1]+dw, -0.5,    gap2[0], 1, 1.5];
# obs4 = [gap1[0]+dw, gap1[1]-dw, -0.5,    gap3[0], gap1[1], 1.5];
# obs5 = [gap3[0]+dw, gap1[1]-dw, -0.5,    1, gap1[1], 1.5];
# obsBounds = [-0.1, -0.1, -0.5, 0, 1.1, 1.5,
#             -0.1, -0.1, -0.5, 1.1, 0, 1.5,
#             -0.1, 1, -0.5, 1.1, 1.1, 1.5,
#             1, -0.1, -0.5, 1.1, 1.1, 1.5,]

# obs = np.concatenate((obs1, obs2, obs3, obs4, obs5, obsBounds), axis=0)
# for i in range(0,obs.shape[0]/(2*dimW)):
#     ax1.add_patch(
#     patches.Rectangle(
#         (obs[i*2*dimW], obs[i*2*dimW+1]),   # (x,y)
#         obs[i*2*dimW+dimW] - obs[i*2*dimW],          # width
#         obs[i*2*dimW+dimW+1] - obs[i*2*dimW+1],          # height
#         alpha=0.6
#     ))
    
# for i in range(0,gridSize*gridSize): # plot occupancy grid
#     cIdx = i + 2*dim
#     if c_sample_seed[cIdx] == 0:
#         plt.scatter(occGridSamples[i,0], occGridSamples[i,1], color="red", s=50, alpha=0.7)
#     else:
#         plt.scatter(occGridSamples[i,0], occGridSamples[i,1], color="green", s=50, alpha=0.7)

# plt.scatter(init[0], init[1], color="red", s=250, edgecolors='black') # init
# plt.scatter(goal[0], goal[1], color="blue", s=250, edgecolors='black') # goal

# plt.show()

# plt.figure(figsize=(10,6), dpi=80)
# viz1 = 1;
# viz2 = 4;
# plt.scatter(y_viz[:,viz1],y_viz[:,viz2], color="green", s=70, alpha=0.1)
# plt.scatter(c_viz[viz1+9],c_viz[viz2+9], color="red", s=250, edgecolors='black') # init
# plt.scatter(c_viz[viz1+9+dim],c_viz[viz2+9+dim], color="blue", s=500, edgecolors='black') # goal
# plt.show()