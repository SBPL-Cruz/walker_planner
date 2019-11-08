# walker_planner
Planner for Wheeled Walker.

To Do
======

* [ ] Modify the cost of an edge to ``c(s, s')  = cost(action) + cost(s')``. Refer to Ben's paper.

Setup
------

1. Clone this repo (unified_planner branch) in catkin_ws/src
Also clone following :
2. git clone https://github.com/shivamvats/smpl.git -b mrmha
3. git clone https://github.com/aurone/leatherman
4. git clone https://github.com/SBPL-Cruz/wheeled_walker

In separate folder :
```
git clone https://github.com/shivamvats/sbpl -b mrmha
mkdir build
mkdir install
cd build
cmake -DCMAKE_INSTALL_PREFIX=../install ..
make install
```

4. Change ~/lolocal in smpl/smpl/CmakeLists.txt and smpl/smpl/smpl-config.cmake.in to install path inside sbpl folder created above

5. catkin build

6.
