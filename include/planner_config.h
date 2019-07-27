#ifndef PLANNER_CONFIG_H
#define PLANNER_CONFIG_H

#include <string>
#include <thread>
#include <vector>
#include <ros/ros.h>

#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <smpl/angles.h>

struct RobotModelConfig
{
    std::string group_name;
    std::vector<std::string> planning_joints;
    std::string kinematics_frame;
    std::string chain_tip_link;
};

struct PlannerConfig
{
    std::string discretization;
    std::string mprim_filename;
    std::string mprim_filenames;
    bool use_multiple_ik_solutions;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_dist_thresh;
    double rpy_snap_dist_thresh;
    double xyzrpy_snap_dist_thresh;
    double short_dist_mprims_thresh;
    double cost_per_cell;
    double inflation_radius;
};

void FillGoalConstraint(
    const std::vector<double>& pose,
    std::string frame_id,
    moveit_msgs::Constraints& goals,
    double xyz_tol,
    double rpy_tol);

bool ReadInitialConfiguration(
    ros::NodeHandle& nh,
    moveit_msgs::RobotState& state);

bool ReadRobotModelConfig(const ros::NodeHandle &nh, RobotModelConfig &config);

bool ReadPlannerConfig(const ros::NodeHandle &nh, PlannerConfig &config);

auto SetupRobotModel(const std::string& urdf, const RobotModelConfig &config)
    -> std::unique_ptr<smpl::KDLRobotModel>;

template <typename T, typename... Args>
inline std::unique_ptr<T> make_unique(Args&&... args){
    return std::move(std::unique_ptr<T>(new T(args...)));
}

#endif
