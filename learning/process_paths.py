#!/usr/bin/env python
# coding: utf-8

# In[2]:


import numpy as np
import dirichlet

from os import listdir
from os.path import isfile, join

# In[3]:


def readPaths(file_names):
    paths = []
    for file_name in file_names:
        path = []
        with open(file_name, 'r') as f:
            header = f.readline()
            state = f.readline()
            while(state):
                path.append([float(val) for val in state.split()])
                state = f.readline()
        paths.append(path)
    return paths
"""Assume that each action modifies only one joint."""
def pathToActions(path):
    actions = []
    # The last action could be a snap motion.
    # Hence ignore.
    for i in range(1, len(path)):
        diff = [x - y for x, y in zip(path[i], path[i-1])]
        actions.append(np.argmax(diff))
    return actions

def actionToActionSpace(actions):
    actionSpace = []
    for action in actions:
        if(action < 3):
            # actionSpace.append(0)
            actionSpace.append('B')
        else:
            # actionSpace.append(1)
            actionSpace.append('A')
    return actionSpace

def compressActionSpaceEncoding(actionSpace):
    compressed = [(actionSpace[0], 1)]
    n = 0
    for a in actionSpace[1:]:
        if not a == compressed[-1][0]:
            compressed.append((a, n))
            n = 0
        n+=1
    return compressed

def toOneHot(data, size):
    data = np.array(data)
    encoding = np.zeros((data.shape[0], size))
    indices = np.arange(data.shape[0])
    encoding[indices, data] = 0.98
    encoding += 0.01
    return encoding


# In[6]:

ONE_EPISODE = 1

file_names = []
if ONE_EPISODE:
    file_names = ["sequence/solution_path_cost.txt",
            "sequence/solution_path_experimental_cost.txt"]
else:
    data_dir = "./sequence/"
    file_names = [join(data_dir, f) for f in listdir(data_dir) if isfile(join(data_dir , f))]

paths = readPaths(file_names)
#all_paths = [state for path in paths for state in path]
#actions = pathToActions(all_paths)
for path in paths:
    actions = pathToActions(path)
    actionSpace = actionToActionSpace(actions)
    compressed = compressActionSpaceEncoding(actionSpace)
    print(compressed)
    #one_hot = toOneHot(actionSpace, 2)
    #print(one_hot)
    #params = dirichlet.mle(one_hot)
    #print(params)


# In[8]:


def testLogitRegression():
    # Feature Space:
    # --------------
    # x - [4, 6]
    # y - [1, 3]
    # arm-retracted - {0, 1}
    # d wrt door - [-2, 2] (initially positive, sign changes on crossing the door)
    # Output Space:
    # -------------
    # output - {0, 1}

    from sklearn.linear_model import LogisticRegression

    # Assume door is at (5, 2)
    def dist(x, y):
        return np.sqrt((x-5)**2 + (y-2)**2)
    xs = np.linspace(4, 6, 8)
    ys = np.linspace(1, 3, 8)
    ds = np.array([dist(x, y) for x, y in zip(xs, ys)])
    print("xs:", xs)
    print("ys:", ys)
    print("ds:", ds)
    retracted = np.zeros_like(ds)
    retracted[ds < 0.5] = 1
    print("retracted:", retracted)
    door_x = np.repeat(5, 8)
    door_y = np.repeat(2, 8)
    #data = np.column_stack((xs, ys, ds, retracted))
    data = np.column_stack((xs, ys, door_x, door_y))
    print("data:", data)
    labels = retracted

    classifier = LogisticRegression(random_state=0, solver='lbfgs',
            multi_class='multinomial').fit(data, labels)
    pred = classifier.predict_proba([[4, 1, 5, 2]])
    print("pred:", pred)
    pred = classifier.predict_proba([[5, 2, 5, 2]])
    print("pred:", pred)
    pred = classifier.predict_proba([[5.2, 2, 5, 2]])
    print("pred:", pred)
    pred = classifier.predict_proba([[6.2, 3, 5, 2]])
    print("pred:", pred)
    print("params:", classifier.get_params())


#testDecisionTree()
#testLogitRegression()


# In[ ]:


#def testDecisionTree():

