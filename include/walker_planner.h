#ifndef WALKER_PLANNER_H
#define WALKER_PLANNER_H

// standard includes
#include <stdlib.h>

//ROS
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

// Library includes
#include <kdl_conversions/kdl_msg.h>
#include <smpl/ros/planner_interface.h>
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

enum PLANNER_MODE{
    A,
    MHA,
    MRMHA
};

class MsgSubscriber {
    public:
        MsgSubscriber(ros::NodeHandle nh, ros::NodeHandle ph);

        void octomapCallback(const octomap_msgs::Octomap& msg);
        void occgridCallback(const nav_msgs::OccupancyGrid& msg);
        void poseCallback(const geometry_msgs::PoseStamped grasp);
        void startCallback(const geometry_msgs::PoseWithCovarianceStamped start);
        int plan(
                ros::NodeHandle nh,
                ros::NodeHandle ph,
                geometry_msgs::Pose grasp,
                octomap_msgs::OctomapWithPose octomap);
        int plan_mrmha(
                ros::NodeHandle nh,
                ros::NodeHandle ph,
                geometry_msgs::Pose grasp,
                octomap_msgs::OctomapWithPose octomap );

        octomap_msgs::OctomapWithPose subscribeOctomap(ros::NodeHandle nh);
        geometry_msgs::Pose subscribeGrasp(ros::NodeHandle nh);
        void publish_path( std::vector<smpl::RobotState> path, ros::Publisher path_pub );

    public:
        ros::NodeHandle m_nh;
        ros::NodeHandle m_ph;
        octomap_msgs::Octomap m_msg;
        octomap_msgs::OctomapWithPose m_map_with_pose;
        geometry_msgs::Pose m_grasp;
        geometry_msgs::Pose m_start_base;
        std::vector<smpl::Vector3> m_occgrid_points;
        std::vector<smpl::RobotState> m_path_cached;
        ros::Publisher m_path_pub;

        ros::Subscriber m_sub_octomap;
        ros::Subscriber m_sub_occgrid;
        ros::Subscriber m_sub_pose;
        ros::Subscriber m_sub_start;

        PLANNER_MODE m_planner_mode {PLANNER_MODE::A};
        bool m_octomap_received;
        bool m_grasp_received;
        bool m_start_received;
        bool m_occgrid_received;
};

#endif
