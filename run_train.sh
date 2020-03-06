#!/bin/bash

# Date: 24th Sept, 2019
# Compare the performance of MR-MHA* with different representations.

home="/usr0/home/svats"
ros="${home}/ros/catkin_ws/src/walker_planner"

SAVE=false

if [ ${SAVE} = true ]; then
    rm "${home}"/.ros/paths/*
    rm "${home}"/.ros/planning_stats.txt
fi

mkdir -p "${home}"/.ros/paths

mode="train"
#mode="val"
instance_id="$1"
#instance_start=$1
#instance_end=$2

#for method in mrmhaplanner_meta_mha_train; do
for method in mrmhaplanner_meta_mha; do
#for method in mrmhaplanner_meta_mha_adm; do
#for method in estimate_fullbody_delta_h; do
#for method in mrmhaplanner; do

    #for instance_id in $(seq $instance_start $instance_end);do

        dataset="${mode}"

        start_path="${ros}/experiments/${dataset}/start_states${instance_id}.txt"

        goal_path="${ros}/experiments/${dataset}/goal_poses${instance_id}.txt"

        env_path="${ros}/experiments/${dataset}/tables_map${instance_id}.env"

        #rosparam set "${method}/robot_start_states_file" ${start_path}
        #rosparam set "${method}/robot_goal_states_file" ${goal_path}
        #rosparam set "${method}/object_filename" ${env_path}

        echo "start_path = ${start_path}"
        echo "goal_path = ${goal_path}"
        echo "object_filename = ${env_path}"

        echo "Calling planner ${method}"
        roslaunch walker_planner ${method}.launch debug:=false instance:=$instance_id start_path:=${start_path} goal_path:=${goal_path} env_path:=${env_path}
        sleep 5

        folder="$(date +%d-%m-%y)/${method}"
        #folder="$(date +%d-%m-%y)/${method}-zero-arm"

        if [ ${SAVE} = true ]; then
            echo "Moving stats"
            mkdir -p "${ros}/results/${mode}/paths"

            #mv "${home}"/.ros/planning_stats.txt "${ros}/results/${folder}/${mode}/"
            #mv "${home}"/.ros/paths/* "${ros}/results/${folder}/${mode}/paths/"
            #mv "${home}"/.ros/train_features$1.txt "${ros}/results/${folder}/${mode}/"
            #mv "${home}"/.ros/train_delta_h$1.txt "${ros}/results/${folder}/${mode}/"

            mv "${home}"/.ros/planning_stats.txt "${ros}/results/${mode}/"
            mv "${home}"/.ros/paths/* "${ros}/results/${mode}/paths/"
            mv "${home}"/.ros/train_features${instance_id}.txt "${ros}/results/${mode}/"
            mv "${home}"/.ros/train_delta_h${instance_id}.txt "${ros}/results/${mode}/"
        fi
    #done
done
