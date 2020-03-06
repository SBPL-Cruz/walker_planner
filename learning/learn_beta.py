import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as stats
import sys
from math import ceil, floor

from utils import readData, writeToFile

def handCodedClusters(data):

    # Euclidean base dist
    data[:, 0] = np.clip(data[:,0], 0, 16)
    data[:,0] = (data[:,0] > 10)

    #BFS End-eff
    #Clip at 20 = 1m
    data[:,1] = np.clip(data[:,1], 0, 16) 
    data[:,1] = (data[:,1] > 2)

    # Circum radius
    data[:,2] = np.clip(data[:,2], 0, 14)
    data[:, 2] = (data[:,2] > 6)

    # Dist to narrow passage
    # Clip to 30 = 1.5m
    data[:,3] = np.clip(data[:,3], 0, 10)
    data[:,3] = (data[:,3] > 3)

    # Let's do two splits each for now
    downSamplingBFSBase = {}
    # upSamplingMapBFSBase = {}
    for i in range(0, 51):
        if i < 25:
            downSamplingBFSBase[i] = 0
        else:
            downSamplingBFSBase[i] = 1

    downSamplingBFSEndEff= {}
    # upSamplingMapBFSEndEff = {}
    for i in range(0, 31):
        if i < 20:
            downSamplingBFSEndEff[i] = 0
        else:
            downSamplingBFSEndEff[i] = 1

    downSamplingBFSCircumRad = {}
    # upSamplingMapBFSCircumRad = {}
    for i in range(0, 21):
        if i < 6:
            downSamplingBFSCircumRad[i] = 0
        else:
            downSamplingBFSCircumRad[i] = 1

    downSamplingBFSDistToNarrowPass = {}
    # upSamplingMapBFSDistToNarrowPass = {}
    for i in range(0, 31):
        if i < 10:
            downSamplingBFSDistToNarrowPass[i] = 0
        else:
            downSamplingBFSDistToNarrowPass[i] = 1

    # for entry in data:
        # entry[0] = downSamplingBFSBase[entry[0]]
        # entry[1] = downSamplingBFSEndEff[entry[1]]
        # entry[2] = downSamplingBFSCircumRad[entry[2]]
        # entry[3] = downSamplingBFSDistToNarrowPass[entry[3]]

    return data

def plotData(data):
    rep0_sum, rep1_sim = [], []
    vals = np.array(data.values())
    rep0_sums, rep1_sums = [], []
    for val in data.values():
        rep0_sums.append(val[0] + val[1])
        rep1_sums.append(val[2] + val[3])
    mean_diff = []
    for val in data.values():
        mean_diff.append( abs( val[0]/ (val[0] + val[1]) - val[2] /(val[2] + val[3]) )  )
    mean_diff = np.array(mean_diff)*100
    print(mean_diff[:10])

    # freq, bins = np.histogram(rep0_sums, bins=100, range=[3, 1000])
    # for b, f in zip(bins[1:], freq):
        # print(round(b, 1), ' '.join(np.repeat('*', f)))
    # freq, bins = np.histogram(rep1_sums, bins=100, range=[3, 1000])
    # for b, f in zip(bins[1:], freq):
        # print(round(b, 1), ' '.join(np.repeat('*', f)))
    freq, bins = np.histogram(mean_diff, bins=100, range=[0, 100])
    for b, f in zip(bins[1:], freq):
        print(round(b, 1), ' '.join(np.repeat('*', f)))

if __name__ == "__main__":
    file_name = "data/context_rewards.txt"
    if len(sys.argv) > 1:
        file_name = sys.argv[1]
    data = readData(file_name)

    NUM_REPS = 2

    alphas = np.zeros( (data.shape[0], NUM_REPS) )
    betas = np.zeros((data.shape[0], NUM_REPS))
    i = 0
    print(data.shape)
    for entry in data:
        for j in range(NUM_REPS):
            alphas[i][j] += entry[-(j+1)]
        i+=1
    # for entry in data:
        # alphas[]

    print("Pre-processing")
    print("Bfs base: min: ", np.min(data[:,0]), " max: ", np.max(data[:, 0]))
    print("Bfs end eff: min: ", np.min(data[:,1]), " max: ", np.max(data[:, 1]))
    print("Yaw dist: min: ", np.min(data[:,2]), " max: ", np.max(data[:, 2]))
    print("D to narrow passage: min: ", np.min(data[:,3]), " max: ", np.max(data[:, 3]))

    print("Rep 0: ", data[data[:,-2] == 0].shape)
    print("Rep 1: ", data[data[:,-2] == 1].shape)

    data = handCodedClusters(data)

    print("Post-processing")
    print("Bfs base: min: ", np.min(data[:,0]), " max: ", np.max(data[:, 0]))
    print("Bfs end eff: min: ", np.min(data[:,1]), " max: ", np.max(data[:, 1]))
    print("Yaw dist: min: ", np.min(data[:,2]), " max: ", np.max(data[:, 2]))
    print("D to narrow passage: min: ", np.min(data[:,3]), " max: ", np.max(data[:, 3]))

    data_dic = {}
    # Record rewards
    # for entry in data:
        # if tuple(entry[:-3]) not in data_dic:
            # data_dic[ tuple(entry[:-3]) ] = entry[-3:]
        # else:
            # data_dic[ tuple(entry[:-3]) ] += entry[-3:]

    # Record alphas and betas
    data_alpha_beta = {}
    for entry in data:
        if tuple(entry[:-2]) not in data_alpha_beta:
            data_alpha_beta[ tuple(entry[:-2]) ] = [1 for i in range(2*NUM_REPS)]
        else:
            arm = entry[-2]
            reward = entry[-1]
            # Update Alpha
            data_alpha_beta[tuple(entry[:-2])][2*arm] += reward
            # Update beta
            data_alpha_beta[tuple(entry[:-2])][2*arm + 1] += (1 - reward)

    plotData(data_alpha_beta)

    confident_alpha_beta = {}
    # Very few Rep0 heuristics
    # We will any way normalize alpha + beta to 10
    # Only ensures that both the representations were sampled
    MIN_PARAM_SUM = 0

    MIN_ADVANTAGE = 0.0
    MAX_C = 500
    for key, val in data_alpha_beta.items():
        mean1 = val[0]/(val[0] + val[1])
        mean2 = val[2]/(val[2] + val[3])
        # if np.sum(val[:-2]) > 20:
        if val[0] + val[1] > MIN_PARAM_SUM and val[2] + val[3] > MIN_PARAM_SUM:
            if abs(mean1 - mean2) > MIN_ADVANTAGE:
                if val[0] + val[1] > MAX_C:
                    val[0] *=  (MAX_C / (val[0] + val[1]))
                    val[1] *=  ( MAX_C / (val[0] + val[1]))
                if val[2] + val[3] > MAX_C:
                    val[2] *=  (MAX_C / (val[2] + val[3]))
                    val[3] *=  (MAX_C / (val[2] + val[3]))
                # val = [ceil(v) for v in val]
                val = [floor(v) for v in val]
                confident_alpha_beta[key] = val
            # confident_alpha_beta[key] = val
    # writeToFile(data_alpha_beta, "beta_prior.txt")
    writeToFile(confident_alpha_beta, "beta_prior-10-00-00.txt")
    # writeToFile(confident_alpha_beta, "beta_prior.txt")

    # for key, val in confident_alpha_beta.items():
        # print(key, val)
    print("Distinct contexts: ", len(data_alpha_beta))
    print("Distinct confident contexts: ", len(confident_alpha_beta) )

    distinct_param_dist = {}
    for key, val in confident_alpha_beta.items():
        distinct_param_dist[tuple(val)] = key
    print("Distinct parameter combos ", len(distinct_param_dist))

    # Reward for every rep.
    plt.scatter(data[:, -1], data[:, -2])
    plt.show()
    plt.scatter(data[:, -0], data[:, -1])
    plt.show()
    plt.scatter(data[:, -0], data[:, -2])
    plt.show()

    # Alphas and betas of every rep
    # plt.scatter(alphas[:,0], betas[:,0], color='r')
    # plt.scatter(alphas[:,1], betas[:,1], color='g')
    # plt.scatter(alphas[:,2], betas[:,2], color='b')
