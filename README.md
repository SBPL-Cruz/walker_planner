# walker_planner
Planner for Wheeled Walker.

Setup
------

1. Clone this repo (unified_planner branch) in catkin_ws/src

Also clone following :
```
git clone https://github.com/shivamvats/smpl.git -b mrmha
git clone https://github.com/aurone/leatherman
git clone https://github.com/SBPL-Cruz/wheeled_walker
```

In separate folder :
```
git clone https://github.com/shivamvats/sbpl -b mrmha
mkdir build
mkdir install
cd build
cmake -DCMAKE_INSTALL_PREFIX=../install ..
make install
```

4. Change ```~/lolocal``` in ```smpl/smpl/CmakeLists.txt``` and ```smpl/smpl/smpl-config.cmake.in``` to install path inside sbpl folder created above

5. Install also :
```
sudo apt-get install ros-kinetic-trac-ik 
```
6. Build only our package: 
```catkin build walker_planner```

Rviz
------
1. Start RVIZ
2. Open the config in the repo proj.rviz
3. Once this open, map, goal states while generating goals and generate plan can be visualized

Generating Traj
--------------
Working directory : walker_planner

1. Generate map in ~/.ros/-multi_room_map.env. Copy this to env folder and rename to proj_env.env : 
```
roslaunch walker_planner generate_map.launch
cp ~/.ros/-multi_room_map.env env/proj_env.env
```

2. Generate start/goal pairs. Generates 500 start/goal pairs in ~/.ros/goal_* ~/.ros/start_*. Copy these to environments folder

```
roslaunch walker_planner generate_start_goals.launch 
cp ~/.ros/goal_* experiments
cp ~/.ros/start_states.txt experiments/
```
3. Change number of paths you need to generate plans for (start/goal pairs) in ```config/walker_right_arm.yaml``` in the ```end_planning_episode variable```. Set to 499 to generate for all start/goal pairs. **By default this is set to 0 to visualize plan for the first start/goal pair only**
4. Run planner and verify in RVIZ :
```roslaunch walker_planner mrmhaplanner.launch```
