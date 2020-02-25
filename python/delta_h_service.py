#!/usr/bin/env python

import sys
import rospy
import pickle
import numpy as np
from walker_planner.srv import DeltaH

model = []

def requestDeltaH(req):
    global model
    print("Service called")

    features = np.array(req.features)
    features = np.reshape(features, (1, features.size))
    delta_h = model.predict(features)

    # Send response
    response = delta_h
    return response

def launchDeltaH(input_args):
    global model

    # Create rospy node
    node_name = 'delta_h_service'
    rospy.init_node(node_name)


    # with open("decision-tree.pkl", 'rb') as f:
    with open("/usr0/home/svats/ros/catkin_ws/src/walker_planner/learning/random-forest.pkl", 'rb') as f:
        model = pickle.load(f)

    # Set up server
    rospy.Service('{}/delta_h'.format(node_name),
                  DeltaH,
                  requestDeltaH)

    print("Service initialized")
    # Main ROS loop
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    launchDeltaH(sys.argv[1:])
