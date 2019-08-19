#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

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

enum PLANNER_ID {
    ARA,
    MHA,
    MRMHA
};
/*
enum PlannerMode {
    STATE_MACHINE,

};

template <typename Planner>
class WalkerPlanner {
    public:
    WalkerPlanner(Planner* _planner);
    ~WalkerPlanner();


}
*/

template <typename CallbackPolicy, typename ExperimentPolicy>
class MotionPlannerROS : public CallbackPolicy, public ExperimentPolicy {
    public:

    MotionPlannerROS( ros::NodeHandle _nh ) :
            CallbackPolicy(_nh), ExperimentPolicy(_nh);
    bool plan( const moveit_msgs::RobotState&, const geometry_msgs::Pose& );

    private:


};


struct Callbacks {

    Callbacks( ros::NodeHandle );
    bool callPlanner() const;

    private:

    void octomapCallback(const octomap_msgs::Octomap& msg);
    void occgridCallback(const nav_msgs::OccupancyGrid& msg);
    void poseCallback(const geometry_msgs::PoseStamped grasp);
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped start);

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
};
