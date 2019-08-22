#ifndef MOTION_PLANNER_ROS_H
#define MOTION_PLANNER_ROS_H

// standard includes
#include <stdlib.h>

//ROS
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

// Library includes
#include <kdl_conversions/kdl_msg.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/RobotState.h>
#include <sbpl/planners/planner.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/heuristic/euclid_fullbody_heuristic.h>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/spatial.h>
#include <smpl/distance_map/edge_euclid_distance_map.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/ros/propagation_distance_field.h>
#include <sbpl_collision_checking/collision_space.h>
#include <smpl/debug/visualizer_ros.h>

//Local
#include "planner_config.h"
#include "collision_space_scene.h"
#include "get_collision_objects.h"

//Publish Path
#include "walker_planner/GraspPose.h"
#include "walker_planner/Path1.h"


using PlanningEpisode = int;

enum PLANNER_ID {
    ARA,
    MHA,
    MRMHA
};

enum ExecutionStatus {
    SUCCESS,
    FAILURE,
    WAITING
};

template < typename SceneUpdatePolicy, typename ExperimentPolicy, typename MotionPlanner >
class MotionPlannerROS : public SceneUpdatePolicy, public ExperimentPolicy {
    public:

    MotionPlannerROS( ros::NodeHandle _nh,
            smpl::KDLRobotModel*,
            std::unique_ptr<CollisionSpaceScene>,
            std::unique_ptr<MotionPlanner>,
            std::shared_ptr<smpl::OccupancyGrid>);
    bool setPlannerParams(const MPlanner::PlannerParams&);
    ExecutionStatus execute(PlanningEpisode);
    std::vector<smpl::RobotState> getPlan(PlanningEpisode);

    private:

    bool updateStart(const moveit_msgs::RobotState&);
    bool updateGoal(const smpl::GoalConstraint&);

    //Get occGrid and objects and add to occupancyGrid.
    std::unique_ptr<MotionPlanner> m_planner_ptr;
    std::vector<MPlanner::PlannerSolution> m_planner_soltns;
    smpl::KDLRobotModel* m_rm_ptr;
};


template <typename SP, typename EP, typename Planner>
MotionPlannerROS<SP, EP, Planner>::MotionPlannerROS(ros::NodeHandle _nh,
            smpl::KDLRobotModel* _rm,
            std::unique_ptr<CollisionSpaceScene> _scene,
            std::unique_ptr<Planner> _planner,
            std::shared_ptr<smpl::OccupancyGrid> _grid_ptr ) :
        SP(_nh, std::move(_scene), _grid_ptr), EP(_nh), m_rm_ptr{_rm},
        m_planner_ptr{std::move(_planner)} {
    ROS_INFO("Initialize visualizer");
    //smpl::VisualizerROS visualizer(_nh, 100);
    //smpl::viz::set_visualizer(&visualizer);

    // Let publishers set up
    //ros::Duration(1.0).sleep();
}

template <typename SP, typename EP, typename Planner>
bool MotionPlannerROS<SP, EP, Planner>::setPlannerParams(const MPlanner::PlannerParams& _params){
    return m_planner_ptr->updatePlannerParams(_params);
}

template <typename SP, typename EP, typename Planner>
ExecutionStatus MotionPlannerROS<SP, EP, Planner>::execute(PlanningEpisode _ep){
    if(this->canCallPlanner()){
        // XXX The map/environment should be updated automatically??
        this->updateMap(_ep);
        updateStart(this->getStart(_ep));
        updateGoal(this->getGoal(_ep));
        MPlanner::PlannerSolution soltn;
        if(!m_planner_ptr->plan(soltn)){
            ROS_WARN("Planning Episode %d Failed", _ep);
            return ExecutionStatus::FAILURE;
        } else{
            m_planner_soltns.push_back(soltn);
            return ExecutionStatus::SUCCESS;
        }
    } else{
        ROS_WARN("Can't call planner");
        return ExecutionStatus::WAITING;
    }
}

template <typename SP, typename EP, typename Planner>
bool MotionPlannerROS<SP, EP, Planner>::updateStart(const moveit_msgs::RobotState& _start){
    smpl::RobotState initial_positions;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            _start.joint_state,
            _start.multi_dof_joint_state,
            m_planner_ptr->robot()->getPlanningJoints(),
            initial_positions,
            missing)){
        ROS_WARN_STREAM("start state is missing planning joints: " << missing);

        moveit_msgs::RobotState fixed_state = _start;
        for (auto& variable : missing) {
            ROS_WARN("  Assume position 0.0 for joint variable '%s'", variable.c_str());
            fixed_state.joint_state.name.push_back(variable);
            fixed_state.joint_state.position.push_back(0.0);
        }

        if (!leatherman::getJointPositions(
                fixed_state.joint_state,
                fixed_state.multi_dof_joint_state,
                m_planner_ptr->robot()->getPlanningJoints(),
                initial_positions,
                missing)){
            return false;
        }
    }

    SP::updateStart(_start, m_rm_ptr);
    return m_planner_ptr->updateStart(initial_positions);
}

template <typename SP, typename EP, typename Planner>
bool MotionPlannerROS<SP, EP, Planner>::updateGoal(const smpl::GoalConstraint& _goal){
    m_planner_ptr->updateGoal(_goal);
}


struct Callbacks {

    Callbacks( ros::NodeHandle,
            std::unique_ptr<CollisionSpaceScene>,
            std::shared_ptr<smpl::OccupancyGrid> );
    bool canCallPlanner() const;
    bool updateMap(PlanningEpisode);
    bool updateStart(const moveit_msgs::RobotState&, smpl::KDLRobotModel*);
    bool updateGoal(const smpl::GoalConstraint&);

    private:

    void octomapCallback(const octomap_msgs::Octomap& msg);
    void occgridCallback(const nav_msgs::OccupancyGrid& msg);
    //void poseCallback(const geometry_msgs::PoseStamped grasp);
    //void startCallback(const geometry_msgs::PoseWithCovarianceStamped start);

    private:

    ros::NodeHandle m_nh;

    std::vector<bool*> m_status_variables;
    bool m_start_received;
    bool m_octomap_received;
    bool m_grasp_received;
    bool m_occgrid_received;

    ros::Subscriber m_sub_octomap;
    ros::Subscriber m_sub_occgrid;
    ros::Subscriber m_sub_pose;
    ros::Subscriber m_sub_start;

    ros::Publisher m_path_pub;

    std::unique_ptr<CollisionSpaceScene> m_collision_scene;
    std::shared_ptr<smpl::OccupancyGrid> m_grid;

    octomap_msgs::Octomap m_msg;
    octomap_msgs::OctomapWithPose m_map_with_pose;
    //geometry_msgs::Pose m_grasp;
    //geometry_msgs::Pose m_start_base;
    std::vector<smpl::Vector3> m_occgrid_points;

    std::vector<smpl::RobotState> m_path_cached;
};


template <class T>
static auto ParseMapFromString(const std::string& s)
    -> std::unordered_map<std::string, T>;

bool IsMultiDOFJointVariable(
    const std::string& name,
    std::string* joint_name,
    std::string* local_name);

std::vector<double> getResolutions(
        smpl::RobotModel* robot,
        const PlannerConfig& params );
#endif
