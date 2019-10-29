#!/bin/bash


#for i in {0..9}; do
#the planner expects the environment to stored at tableObstacles.yaml
#so move the numbered one to this path

#for method in unconstrained_mha focal_mha mha_plus original_mha; do
#for method in focal_mha; do
#for method in gbfs; do
home="/usr0/home/svats"
ros="${home}/ros/catkin_ws/src/walker_planner"

rm "${home}"/.ros/paths/*
rm "${home}"/.ros/planning_stats.txt

for method in motion_planner ; do #mha_planner mrmha_planner motion_planner; do
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
    elif [ ${method} = "motion_planner" ]; then
        folder="mrmha-intelligent"
    fi
    mv "${home}"/.ros/planning_stats.txt "${ros}"/results/${folder}/
    mkdir -p "${ros}"/results/${folder}/paths/
    mv "${home}"/.ros/paths/* "${ros}"/results/${folder}/paths
done
