#!/bin/bash

home="/usr0/home/svats"
ros="${home}/ros/catkin_ws/src/walker_planner"

for i in $(seq 26 50); do
    instance_id=$i

    env_file_name="tables_map${instance_id}.env"

    roslaunch walker_planner generate_map.launch debug:=false file_name:="${env_file_name}"

    sleep 2

    rosparam set instance_id ${instance_id}

    roslaunch walker_planner generate_start_goals.launch debug:=false instance:="${instance_id}" env_path:="${env_file_name}"

    sleep 2

done

