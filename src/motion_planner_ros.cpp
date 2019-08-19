#include <algorithm>
#include "motion_planner_ros.h"

using namespace smpl;

Callbacks::Callbacks(ros::NodeHandle _nh) : m_nh{_nh} {
    m_start_received = false;
    m_octomap_received = false;
    m_grasp_received = false;
    m_occgrid_received = false;

    ros::param::set("/walker_planner_request", 0);
    ros::param::set("/walker_planner_done", 0);
    m_path_pub = m_nh.advertise<walker_planner::Path1>("Robot_path", 1000);
    m_sub_start = m_nh.subscribe("/poseupdate", 1000, &MsgSubscriber::startCallback, this);
    //m_sub_occgrid = m_nh.subscribe("/map", 1000, &MsgSubscriber::occgridCallback, this);
    //m_sub_octomap = m_nh.subscribe("/octomap_binary", 1000, &MsgSubscriber::octomapCallback, this);
    m_sub_pose = m_nh.subscribe("/Grasps", 1000, &MsgSubscriber::poseCallback, this);

    m_status_variables = { &m_start_received,
            &m_octomap_received,
            &m_occgrid_received,
            &m_grasp_received };
}

bool Callbacks::callPlanner() const {
    return std::all_of(
            m_status_variables.begin(), m_status_variables.end(),
            [](bool* x){
                return *x;
                } );
}
