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

#mode="debug/ik"
mode="val"
#for method in mrmhaplanner_cobandits; do
for method in mrmhaplanner; do
#for method in mrmhaplanner_meta_a; do
#for method in mrmhaplanner_hdts; do
    for hardness in easy1; do
        for plan_length in short; do
        #for plan_length in short; do

            dataset="${mode}/${hardness}/${plan_length}"
            #dataset="${mode}"
            start_path="${ros}/experiments/${dataset}/start_states.txt"
            goal_path="${ros}/experiments/${dataset}/goal_poses.txt"
            env_path="${ros}/experiments/${mode}/${hardness}/multi_room_map.env"
            #env_path="${ros}/experiments/${mode}/multi_room_map.env"
            beta_prior_path="${ros}/learning/${beta_prior}.txt"
            rosparam set "${method}/robot_start_states_file" ${start_path}
            rosparam set "${method}/robot_goal_states_file" ${goal_path}
            rosparam set "${method}/object_filename" ${env_path}
            rosparam set "${method}/beta_prior_file" "${beta_prior_path}"
            echo "start_path = ${start_path}"
            echo "goal_path = ${goal_path}"
            echo "object_filename = ${env_path}"

            echo "Calling planner ${method}"
            roslaunch walker_planner ${method}.launch debug:=false
            sleep 5

            folder="$(date +%d-%m-%y)/${method}/h6-nonuniform-g-uniform-h"

            if [ ${SAVE} = true ]; then
                echo "Moving stats"
                mkdir -p "${ros}/results/${folder}/${mode}/paths"

                mv "${home}"/.ros/planning_stats.txt "${ros}/results/${folder}/${mode}/"
                mv "${home}"/.ros/paths/* "${ros}/results/${folder}/${mode}/paths/"
            fi
        done
    done
done
