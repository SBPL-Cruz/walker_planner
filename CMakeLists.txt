cmake_minimum_required(VERSION 2.8.3)
project(walker_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

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
  trac_ik_robot_model
  smpl_ompl_interface
  smpl_moveit_interface
  visualization_msgs
)


find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS unit_test_framework)
find_package(OMPL REQUIRED)
find_package(smpl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GSL REQUIRED)
#list(APPEND CMAKE_PREFIX_PATH "~/lolocal")
#find_package(hsearch REQUIRED)

add_definitions(-DSV_PACKAGE_NAME="walker_planner")
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})
#include_directories(SYSTEM "~/lolocal/include/gsl")
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

set( PLANNERS
    src/planners/rmhaplanner.cpp)
    #src/planners/mrmhaplanner.cpp)

set (AUXILIARY
    src/utils/utils.cpp)

add_executable(collision_model_visualizer src/debug/collision_model_visualizer.cpp
    src/config/collision_space_scene.cpp)
target_link_libraries(collision_model_visualizer ${catkin_LIBRARIES} smpl::smpl)

#add_executable( test_motion_planner
#    src/mha/test_motion_planner.cpp
#    src/motion_planner_ros.cpp
#    src/config/collision_space_scene.cpp
#    src/config/planner_config.cpp
#    src/config/get_collision_objects.cpp)
#target_link_libraries(test_motion_planner ${catkin_LIBRARIES} smpl::smpl
#    ${orocos_kdl_LIBRARIES} GSL::gsl GSL::gslcblas)
#add_dependencies( test_motion_planner ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${catkin_EXPORTED_TARGETS} )
#
#add_executable( test_mrmha_planner
#    src/mha/test_mrmha_planner.cpp
#    src/motion_planner_ros.cpp
#    src/config/collision_space_scene.cpp
#    src/config/planner_config.cpp
#    src/config/get_collision_objects.cpp)
#target_link_libraries(test_mrmha_planner ${catkin_LIBRARIES} smpl::smpl
#    ${orocos_kdl_LIBRARIES})
#add_dependencies( test_mrmha_planner ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${catkin_EXPORTED_TARGETS} )
#
#add_executable( test_mrmha_rr_planner
#    src/mha/test_mrmha_rr_planner.cpp
#    #src/planners/mrmhaplanner.cpp
#    src/motion_planner_ros.cpp
#    src/config/collision_space_scene.cpp
#    src/config/planner_config.cpp
#    src/config/get_collision_objects.cpp
#    ${PLANNERS})
#target_link_libraries(test_mrmha_rr_planner ${catkin_LIBRARIES} smpl::smpl
#    ${orocos_kdl_LIBRARIES})
#add_dependencies( test_mrmha_rr_planner ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${catkin_EXPORTED_TARGETS} )

add_executable( mrmhaplanner
    src/mha/mrmhaplanner.cpp
    src/motion_planner_ros.cpp
    src/config/planner_config.cpp
    src/config/collision_space_scene.cpp
    src/config/get_collision_objects.cpp
    )
target_link_libraries( mrmhaplanner ${catkin_LIBRARIES} smpl::smpl )

add_executable( walker_planner
    src/mha/walker_planner.cpp
    src/motion_planner_ros.cpp
    src/config/collision_space_scene.cpp
    src/config/planner_config.cpp
    src/config/get_collision_objects.cpp)
target_link_libraries(walker_planner ${catkin_LIBRARIES} smpl::smpl ${orocos_kdl_LIBRARIES})
add_dependencies( walker_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_executable( test_mha_planner
    src/mha/test_mha_planner.cpp
    src/motion_planner_ros.cpp
    src/config/collision_space_scene.cpp
    src/config/planner_config.cpp
    src/config/get_collision_objects.cpp)
target_link_libraries(test_mha_planner ${catkin_LIBRARIES} smpl::smpl ${orocos_kdl_LIBRARIES})
add_dependencies( test_mha_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_executable( test_rmha_planner
    src/mha/test_rmha_planner.cpp
    src/motion_planner_ros.cpp
    src/config/collision_space_scene.cpp
    src/config/planner_config.cpp
    src/config/get_collision_objects.cpp
    ${PLANNERS})
target_link_libraries(test_rmha_planner ${catkin_LIBRARIES} smpl::smpl ${orocos_kdl_LIBRARIES})
add_dependencies( test_rmha_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_executable( test_ompl_planner
    src/test_ompl_planner.cpp
    src/config/collision_space_scene.cpp
    src/config/get_collision_objects.cpp
    src/config/planner_config.cpp)
target_link_libraries( test_ompl_planner ${catkin_LIBRARIES} smpl::smpl )

add_executable(generate_map
    src/utils/generate_map.cpp
    src/config/planner_config.cpp
    src/config/get_collision_objects.cpp
    ${AUXILIARY})
target_link_libraries(generate_map ${catkin_LIBRARIES} smpl::smpl)

add_executable(generate_start_goals
    src/utils/generate_start_goals.cpp
    src/utils/start_goal_generator.cpp
    src/config/planner_config.cpp
    src/config/collision_space_scene.cpp
    src/config/get_collision_objects.cpp
    ${AUXILIARY})
target_link_libraries(generate_start_goals ${catkin_LIBRARIES} smpl::smpl)

add_executable( test_ik
    test/test_ik.cpp
    src/config/collision_space_scene.cpp
    src/config/planner_config.cpp
    src/config/get_collision_objects.cpp)
target_link_libraries(test_ik ${catkin_LIBRARIES} smpl::smpl ${orocos_kdl_LIBRARIES})
add_dependencies( test_ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

#add_executable( test_templated_mrmha_planner
#    test/test_mrmhaplanner.cpp
#    src/motion_planner_ros.cpp
#    src/config/planner_config.cpp
#    src/config/collision_space_scene.cpp
#    src/config/get_collision_objects.cpp
#    )
#target_link_libraries( test_templated_mrmha_planner ${catkin_LIBRARIES} smpl::smpl )

add_executable( test_start_goal_generator
    test/test_start_goal_generator.cpp src/utils/start_goal_generator.cpp)
target_link_libraries(test_start_goal_generator ${catkin_LIBRARIES} smpl::smpl)

add_executable( test_fk
    test/test_fk.cpp
    src/config/planner_config.cpp
    src/config/collision_space_scene.cpp
    src/motion_planner_ros.cpp
    src/config/get_collision_objects.cpp)
target_link_libraries(test_fk ${catkin_LIBRARIES} smpl::smpl)

add_executable( debug_planner
    src/debug/debug_planner.cpp
    src/config/planner_config.cpp
    src/config/collision_space_scene.cpp
    src/motion_planner_ros.cpp
    src/config/get_collision_objects.cpp)
target_link_libraries(debug_planner ${catkin_LIBRARIES} smpl::smpl)
