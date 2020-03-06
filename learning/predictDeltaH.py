import sys
import argparse
import math
import pickle

import numpy as np
import sklearn as skl
from sklearn.model_selection import train_test_split
from sklearn.linear_model import SGDRegressor
from sklearn import svm
from sklearn.linear_model import LinearRegression
from sklearn import tree
from sklearn.ensemble import RandomForestRegressor, ExtraTreesRegressor
from sklearn.neural_network import MLPRegressor
from sklearn.metrics import mean_squared_error
from utils import readData
import matplotlib.pyplot as plt

FEATURES_FILE = "data/train_features.txt"
VALUES_FILE = "data/train_delta_h.txt"
SPLIT = 0.8

def filterData(X, Y):
    THRESH = 100
    filtered_X, filtered_Y = [], []
    print("Outliers: ", (Y[Y > 100]).size)
    for x, y in zip(X, Y):
        if math.isnan(y):
            continue
        elif (y < THRESH):
        # else:
            filtered_X.append(x)
            filtered_Y.append(y)
        elif (y >= THRESH):
            filtered_X.append(x)
            filtered_Y.append(THRESH)

    return ( np.array(filtered_X), np.array(filtered_Y) )

def loadData(normalize=False):
    global FEATURES_FILE
    global VALUES_FILE
    global SPLIT

    features = []
    min_feature_size = float('inf')
    with open(FEATURES_FILE) as f:
        line = f.readline()
        while(line):
            features.append([float(val) for val in line.split()])
            if(len(features[-1]) < min_feature_size):
                min_feature_size = len(features[-1])
            line = f.readline()
    # Make feature vector size equal
    truncated_features = []
    for feat in features:
        truncated_features.append(feat[-min_feature_size:])
        # truncated_features.append(feat[:min_feature_size])

    truncated_features = np.array(truncated_features)[:(int)(1.0*len(truncated_features))]
    print("Size of features: ", truncated_features.shape)

    values = []
    with open(VALUES_FILE) as f:
        for line in f.readlines():
            values.append(float(line.split()[0]))
    values = np.array(values)[:(int)(1.0*len(values))]

    truncated_features, values = filterData(truncated_features, values)
    u, c = np.unique(values, return_counts=True)
    print("Unique counter: ", u.shape)
    # values[values > 1000] = 1000
    plt.hist(values)
    plt.show()
    print("Size of values: ", values.shape)

    scale = 1.0 #np.max(values)
    if(normalize):
        truncated_features = truncated_features - np.mean(truncated_features,
                axis=0)
        truncated_features = truncated_features / np.std(truncated_features,
                axis=0)
        # values = values - np.mean(values, axis=0)
        # values = values / np.std(values, axis=0)
        # values = 100.0 * values / np.max(values)
        # values = values / np.max(values)
        # values = values / 100
        pass

    return (train_test_split(
            truncated_features, values, test_size=1 - SPLIT, shuffle=True),
            scale)

def testModel(model, X_test, y_test, scale, plot=False):
    preds = model.predict(X_test)
    rmse = np.sqrt(mean_squared_error(y_test, preds))
    print("Test RMSE: ", rmse)
    # print(plot)
    for i in range(20):
        print(scale*y_test[i], scale*preds[i])
    if(plot):
        preds[preds < 0] = 0
        plt.scatter(y_test, preds)
        plt.xlabel("Ground Truth")
        plt.ylabel("Predicted")
        plt.plot([0, 1], [0, 1], 'r')
        # plt.plot(preds/y_test, 'bo')
        # plt.plot(y_test, 'r')
        plt.show()
    return rmse


if __name__ == "__main__":
    print("Check data files in the script")
    # global FEATURES_FILE
    # global VALUES_FILE

    # FEATURES_FILE = sys.argv[1]
    # VALUES_FILE = sys.argv[2]

    parser = argparse.ArgumentParser()
    parser.add_argument('--plot', help="Generate plots")
    parser.add_argument('--save', help="Save trained model")
    args = parser.parse_args()
    print(args)

    (X_train, X_test, y_train, y_test), scale = loadData(normalize=True)
    print("Train size: ", y_train.shape)
    print("Test size: ", y_test.shape)

    # reg = LinearRegression(normalize=True).fit(X_train, y_train)
    # reg = SGDRegressor(loss="squared_loss", penalty="l2", max_iter=100)
    # reg = svm.SVR()
    # reg = tree.DecisionTreeRegressor()
    # reg = MLPRegressor(hidden_layer_sizes=(10,))

    regs = {
            # "linear-regression" : LinearRegression(normalize=True).fit(X_train, y_train),
            # "sgd" : SGDRegressor(loss="squared_loss", penalty="l1", max_iter=1000),
            # "svm" : svm.SVR(gamma="scale"),
            # "decision-tree" : tree.DecisionTreeRegressor(max_depth=20),
            # "random-forest" : RandomForestRegressor(max_depth=8, n_estimators=100, max_features=None),
            # "random-forest" : RandomForestRegressor(max_depth=5, n_estimators=100, max_features=None),
            # "extra-trees" : ExtraTreesRegressor(max_depth=10, n_estimators=100, max_features=None),
            "mlp" : MLPRegressor(hidden_layer_sizes=(300, 100 ), learning_rate='adaptive', max_iter=500)
            }

    errors = []
    for name, reg in regs.items():
    # for depth in range(2, 25, 2):
        # name  = "random-forest"
        # reg = RandomForestRegressor(max_depth=depth)
        print(name)
        print("------")
        reg.fit(X_train, y_train)
        train_rmse = np.sqrt(mean_squared_error(y_train, reg.predict(X_train)))
        # plt.plot(reg.predict(X_train), 'b')
        # plt.plot(y_train, 'r')
        # plt.show()

        print("Train RMSE: ", train_rmse)
        plot = False
        if( args.plot in ['True', 'true'] ):
            plot = True
        errors.append(testModel(reg, X_test, y_test, scale, plot=plot))
        if(args.save in ['True', 'true']):
            with open(name + ".pkl", 'wb') as f:
                pickle.dump(reg, f)
    # plt.plot(range(2, 25, 2), errors)
    # plt.show()

    # testModel(reg, X_train, y_train)
