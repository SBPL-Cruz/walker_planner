#!/bin/bash

# Date: 24th Sept, 2019
# Compare the performance of MR-MHA* with different representations.

home="/usr0/home/svats"
ros="${home}/ros/catkin_ws/src/walker_planner"

rm "${home}"/.ros/paths/*
rm "${home}"/.ros/planning_stats.txt

mkdir -p "${home}"/.ros/paths

mode="train"
#for method in mrmhaplanner_cobandits_train; do
for method in mrmhaplanner_cobandits; do
#for method in mrmhaplanner_rdts; do
    for hardness in easy; do 
        for plan_length in long; do
        #for plan_length in short; do
            #for beta_prior in beta_prior-1-02-03 beta_prior-1-05-02 beta_prior-1-05-03 beta_prior-1-10-01; do
            for beta_prior in beta_prior-10-00-00; do

                dataset="${mode}/${hardness}/${plan_length}"
                start_path="${ros}/experiments/${dataset}/start_states.txt"
                goal_path="${ros}/experiments/${dataset}/goal_poses.txt"
                env_path="${ros}/experiments/${mode}/${hardness}/multi_room_map.env"
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

                #when the trials are done, kill the planner
                #screen -r planner
                sleep 10

                #save the stats
                folder=""
                #suffix="-base-arm-full"
                #suffix="-base-arm"
               suffix="-${beta_prior}"
                if [ ${method} = "mhaplanner" ]; then
                    folder="mha"
                elif [ ${method} = "mrmhaplanner" ]; then
                    folder="mrmha-rr"
                elif [ ${method} = "mrmhaplanner_dts" ]; then
                    folder="mrmha-dts"
                elif [ ${method} = "mrmhaplanner_ucb" ]; then
                    folder="mrmha-ucb"
                elif [ ${method} = "mrmhaplanner_cobandits" ]; then
                    folder="mrmha-cobandits"
                #elif [ ${method} = "rmha_planner" ]; then
                    #folder="rmha"
                #elif [ ${method} = "mrmha_rr_planner" ]; then
                    #folder="mrmha_rr"
                #elif [ ${method} = "motion_planner" ]; then
                    #folder="mrmha-intelligent"
                #elif [ ${method} = "mrmha_planner_1" ]; then
                    #folder="mrmha1"
                #elif [ ${method} = "mrmha_planner_2" ]; then
                    #folder="mrmha2"
                #elif [ ${method} = "mrmha_planner_3" ]; then
                    #folder="mrmha3"
                fi
                folder="${folder}${suffix}"

                mkdir -p "${ros}/results/${folder}/${hardness}/${plan_length}/paths"
                rm "${ros}/results/${folder}/${hardness}/${plan_length}/planning_stats.txt"
                mv "${home}"/.ros/planning_stats.txt "${ros}/results/${folder}/${hardness}/${plan_length}/"
                mv "${home}"/.ros/paths/* "${ros}/results/${folder}/${hardness}/${plan_length}/paths/"
            done
        done
    done
done
