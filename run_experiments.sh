#!/bin/bash

# Date: 24th Sept, 2019
# Compare the performance of MR-MHA* with different representations.

home="/usr0/home/svats"
ros="${home}/ros/catkin_ws/src/walker_planner"

rm "${home}"/.ros/paths/*
rm "${home}"/.ros/planning_stats.txt

#for method in mha_planner mrmha_planner_1 mrmha_planner_2 mrmha_planner_3 ; do
#for method in mrmha_rr_planner mha_planner ; do
for method in mha_planner rmha_planner rmha_planner rmha_planner rmha_planner rmha_planner; do
    #mkdir -p stats
    #$mkdir -p stats/paths_${method}_${i}

    #if [ $method == "rrt" ]; then
        #rosrun monolithic_pr2_planner_node runTests experiments/fbp_tests$i.yaml $method
    #else
    echo "Calling planner ${method}"
    roslaunch walker_planner test_${method}.launch
    #fi

    #when the trials are done, kill the planner
    #screen -r planner
    sleep 10

    #save the stats
    folder=""
    if [ ${method} = "mha_planner" ]; then
        folder="mha"
    elif [ ${method} = "mrmha_planner" ]; then
        folder="mrmha"
    elif [ ${method} = "rmha_planner" ]; then
        folder="rmha"
    elif [ ${method} = "mrmha_rr_planner" ]; then
        folder="mrmha_rr"
    elif [ ${method} = "motion_planner" ]; then
        folder="mrmha-intelligent"
    elif [ ${method} = "mrmha_planner_1" ]; then
        folder="mrmha1"
    elif [ ${method} = "mrmha_planner_2" ]; then
        folder="mrmha2"
    elif [ ${method} = "mrmha_planner_3" ]; then
        folder="mrmha3"
    fi

    if [ ${method} = "mha_planner" ]; then
        mv "${home}"/.ros/planning_stats.txt "${ros}"/results/${folder}/
        mkdir -p "${ros}"/results/${folder}/paths/
        mv "${home}"/.ros/paths/* "${ros}"/results/${folder}/paths/
    fi
done
mv "${home}"/.ros/planning_stats.txt "${ros}"/results/rmha/
mkdir -p "${ros}"/results/${folder}/paths/
mv "${home}"/.ros/paths/* "${ros}"/results/rmha/paths
