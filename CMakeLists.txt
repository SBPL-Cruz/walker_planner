cmake_minimum_required(VERSION 2.8.3)
project(walker_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  nav_msgs
  geometry_msgs
  leatherman
  moveit_core
  moveit_msgs
  moveit_ros_planning
  roscpp
  sbpl_collision_checking
  sbpl_kdl_robot_model
  smpl_moveit_interface
  visualization_msgs
)


find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS unit_test_framework)
find_package(OMPL REQUIRED)
find_package(smpl REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(hsearch REQUIRED)

add_definitions(-DSV_PACKAGE_NAME="walker_planner")
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})
#include_directories(SYSTEM ${CATKIN_DEVEL_PREFIX}/include})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
add_message_files(
    FILES
    GraspPose.msg
    Path1.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    )

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES walker
CATKIN_DEPENDS geometry_msgs leatherman moveit_msgs moveit_ros_planning message_runtime std_msgs
#orocos_kdl roscpp sbpl_collision_checking sbpl_kdl_robot_model smpl
#smpl_ompl_interface smpl_moveit_interface visualization_msgs
#DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(collision_model_visualizer src/collision_model_visualizer.cpp
    src/collision_space_scene.cpp)
target_link_libraries(collision_model_visualizer ${catkin_LIBRARIES} smpl::smpl)

add_executable(walker_interface_planner src/walker_interface_planner.cpp src/collision_space_scene.cpp)
add_dependencies(walker_interface_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(walker_interface_planner ${catkin_libraries} smpl::smpl ${orocos_kdl_libraries})

add_executable(interactive_walker_model src/interactive_walker_model.cpp src/collision_space_scene.cpp)
add_dependencies(interactive_walker_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${CATKIN_EXPORTED_TARGETS})
target_link_libraries(interactive_walker_model ${catkin_libraries} smpl::smpl ${orocos_kdl_libraries})

#add_executable(walker_interface_mrmha src/walker_interface_mrmha.cpp src/collision_space_scene.cpp)
#target_link_libraries(walker_interface_mrmha ${catkin_LIBRARIES} smpl::smpl ${orocos_kdl_LIBRARIES})
#add_dependencies(walker_interface_mrmha ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(walker_planner src/walker_planner.cpp src/collision_space_scene.cpp)
#target_link_libraries(walker_planner ${catkin_LIBRARIES} smpl::smpl)

#add_executable(interactive_planner src/interactive_planner.cpp src/collision_space_scene.cpp)
#target_link_libraries(interactive_planner ${catkin_LIBRARIES} smpl::smpl)

