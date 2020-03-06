#!/bin/bash

# Date: 24th Sept, 2019
# Compare the performance of MR-MHA* with different representations.

home="/usr0/home/svats"
ros="${home}/ros/catkin_ws/src/walker_planner"

SAVE=true

if [ ${SAVE} = true ]; then
    rm "${home}"/.ros/paths/*
    rm "${home}"/.ros/planning_stats.txt
fi

mkdir -p "${home}"/.ros/paths

#mode="debug/ik"
#mode="val"
mode="test"
#instance_id="1"

for method in mrmhaplanner; do
#for method in mrmhaplanner_meta_a; do
#for method in mrmhaplanner_meta_mha; do

    dataset="${mode}"
    #for instance_id in "1" "2" "3" "4" "5"; do
    for instance_id in "26" "27" "28" "29" "30"; do

        start_path="${ros}/experiments/${dataset}/start_states${instance_id}.txt"
        goal_path="${ros}/experiments/${dataset}/goal_poses${instance_id}.txt"
        env_path="${ros}/experiments/${dataset}/tables_map${instance_id}.env"

        rosparam set "${method}/robot_start_states_file" ${start_path}
        rosparam set "${method}/robot_goal_states_file" ${goal_path}
        rosparam set "${method}/object_filename" ${env_path}

        echo "start_path = ${start_path}"
        echo "goal_path = ${goal_path}"
        echo "object_filename = ${env_path}"

        echo "Calling planner ${method}"
        roslaunch walker_planner ${method}.launch debug:=false instance:=${instance_id} start_path:=${start_path} goal_path:=${goal_path} env_path:=${env_path}
        sleep 5
        #folder="${method}" #"/$(date +%d-%m-%y)"
        folder="mhaplanner"
        if [ ${SAVE} = true ]; then
            echo "Moving stats"
            mkdir -p "${ros}/results/${mode}/${folder}/paths${instance_id}"

            mv "${home}"/.ros/planning_stats.txt "${ros}/results/${mode}/${folder}/planning_stats${instance_id}.txt"
            mv "${home}"/.ros/paths/* "${ros}/results/${mode}/${folder}/paths${instance_id}/"
        fi

    done


done
