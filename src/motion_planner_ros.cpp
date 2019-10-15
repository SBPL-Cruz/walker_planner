#include <algorithm>
#include "motion_planner.h"
#include "motion_planner_ros.h"

using namespace smpl;

Callbacks::Callbacks(ros::NodeHandle _nh,
        CollisionSpaceScene* _scene,
        smpl::OccupancyGrid* _grid_ptr )
    : m_nh{_nh}, m_collision_scene{_scene}, m_grid{_grid_ptr} {

    //m_start_received = false;
    m_octomap_received = false;
    //m_grasp_received = false;
    m_occgrid_received = false;

    ros::param::set("/walker_planner_request", 0);
    ros::param::set("/walker_planner_done", 0);
    m_path_pub = m_nh.advertise<walker_planner::Path1>("Robot_path", 1000);
    //m_sub_occgrid = m_nh.subscribe("/map", 1000, &Callbacks::occgridCallback, this);
    m_sub_octomap = m_nh.subscribe("/octomap_binary", 1000, &Callbacks::octomapCallback, this);
    //m_sub_start = m_nh.subscribe("/poseupdate", 1000, &Callbacks::startCallback, this);
    //m_sub_pose = m_nh.subscribe("/Grasps", 1000, &Callbacks::poseCallback, this);

    m_status_variables = {
            &m_octomap_received,
            //&m_occgrid_received
            };
}

bool Callbacks::canCallPlanner() const {
    return std::all_of(
            m_status_variables.begin(), m_status_variables.end(),
            [](bool* x){
                return *x;
                } );
}

bool Callbacks::updateMap(PlanningEpisode _ep){
    if(m_collision_scene->ProcessOctomapMsg(m_map_with_pose)){
        ROS_INFO("Succesfully added octomap");
    }
    else{
        ROS_INFO("Could not added octomap");
        return false;
    }
    //auto map_config = getMultiRoomMapConfig(m_nh);
    //std::vector<moveit_msgs::CollisionObject> tmp;
    //auto objects = GetMultiRoomMapCollisionCubes(m_grid->getReferenceFrame(), map_config, tmp);
    //for (auto& object : objects) {
    //    m_collision_scene->ProcessCollisionObjectMsg(object);
    //}

    assert(m_grid != nullptr);
    m_grid->addPointsToField(m_occgrid_points);

    SV_SHOW_INFO(m_collision_scene->getOccupiedVoxelsVisualization());

    return true;
}

bool Callbacks::updateStart(const moveit_msgs::RobotState& _start,
        smpl::KDLRobotModel* _rm_ptr){
    // Set reference state in the robot planning model...
    auto marker = m_collision_scene->getCollisionRobotVisualization(_start.joint_state.position);
    for(auto& m : marker.markers)
        m.ns = "Start state";
    SV_SHOW_INFO(marker);
    smpl::urdf::RobotState reference_state;

    InitRobotState(&reference_state, &_rm_ptr->m_robot_model);
    for (auto i = 0; i < _start.joint_state.name.size(); ++i) {
        auto* var = GetVariable(&_rm_ptr->m_robot_model, &_start.joint_state.name[i]);
        if (var == NULL) {
            ROS_WARN("Failed to do the thing");
            continue;
        }
        ROS_INFO("Set joint %s to %f", _start.joint_state.name[i].c_str(), _start.joint_state.position[i]);
        SetVariablePosition(&reference_state, var, _start.joint_state.position[i]);
    }
    SetReferenceState(_rm_ptr, GetVariablePositions(&reference_state));
    return m_collision_scene->SetRobotState(_start);
}

void Callbacks::octomapCallback(const octomap_msgs::Octomap& msg) {
    ROS_ERROR("Octomap received");
    m_map_with_pose.header.frame_id = "dummy_base";
    m_map_with_pose.octomap = msg;

    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    geometry_msgs::Quaternion q;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    q.w = 1;
    geometry_msgs::Pose pose;
    pose.position = p;
    pose.orientation = q;

    m_map_with_pose.origin = pose;
    m_octomap_received = true;
}

void Callbacks::occgridCallback(const nav_msgs::OccupancyGrid& msg) {
    ROS_ERROR("Occupancy Grid received");

    std::vector<smpl::Vector3> points;
    int height = msg.info.height;
    int width = msg.info.width;
    auto res = msg.info.resolution;
    geometry_msgs::Point origin = msg.info.origin.position;
    ROS_ERROR("Occupancy grid origin: %f %f", origin.x, origin.y);
    ROS_ERROR("Occupancy grid res: %f", res);

    std::string planning_mode = "FULLBODY";
    ros::param::get("/walker_planner_mode", planning_mode);


    // XXX Assume res of incoming occGrid is 0.1.
    //int scale = 0.1 / 0.02;
    //int scaled_i=0, scaled_j=0;
    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {
            int val = msg.data[i*width + j];
            if (val > 50) {
                //scaled_i = i*scale, scaled_j = j*scale;
                //for (int k=scaled_i; k < scaled_i+6; k++) {
                    for (int h=0; h < 0.65/res; h++) {
                        smpl::Vector3 v,v2,v3,v4,v5;
                        v[0] = j*res + origin.x;//k;
                        v[1] = i*res + origin.y;//l;
                        v[2] = h*res;
                        v2[0] = (j-1)*res + origin.x;//k;
                        v2[1] = (i-1)*res + origin.y;//l;
                        v2[2] = h*res;
                        v3[0] = (j+1)*res + origin.x;//k;
                        v3[1] = (i+1)*res + origin.y;//l;
                        v3[2] = h*res;
                        v4[0] = (j-2)*res + origin.x;//k;
                        v4[1] = (i-2)*res + origin.y;//l;
                        v4[2] = h*res;
                        v5[0] = (j+2)*res + origin.x;//k;
                        v5[1] = (i+2)*res + origin.y;//l;
                        v5[2] = h*res;


                        points.push_back(v);
                    }
            }
        }
    }
    m_occgrid_points = points;
    m_occgrid_received = true;
    ROS_INFO("Occupancy Grid loaded.");
}

/*
void Callbacks::poseCallback(const geometry_msgs::PoseStamped grasp) {
    m_grasp = grasp.pose;
    m_grasp_received = true;

    std::string planning_mode = "FULLBODY";
    ros::param::get("/walker_planner_mode", planning_mode);
    if( ( planning_mode == "FULLBODY" ) && m_start_received){// && m_occgrid_received &&  m_octomap_received) {
        m_octomap_received = false;
        m_start_received = false;
        m_grasp_received = false;

        ros::param::set("/test_walker_interface/robot_model/chain_tip_link", "right_palm_link");
        ros::param::set("/test_walker_interface/robot_model/planning_joints", "x  y theta right_j1  right_j2 right_j3 right_j4 right_j5 right_j6 right_j7" );
        std::string pkg_path = ros::package::getPath("walker_planner");
        ros::param::set("/test_walker_interface/planning/mprim_filename", pkg_path + "/config/walker.mprim");

        plan(m_nh, m_ph, m_grasp);
    }
    else if( ( planning_mode == "BASE" ) && m_start_received  &&
            m_occgrid_received ) {
        ROS_ERROR("Base Planner Called.");

        ros::param::set("/test_walker_interface/robot_model/chain_tip_link", "base_link");
        ros::param::set("/test_walker_interface/robot_model/planning_joints", "x y theta");
        std::string pkg_path = ros::package::getPath("walker_planner");
        ros::param::set("/test_walker_interface/planning/mprim_filename", pkg_path + "/config/walker_base.mprim");
        plan(m_nh, m_ph, m_grasp);
    }
    else{
        //ROS_ERROR("Planner mode not understood.");
    }
}

void Callbacks::startCallback(const geometry_msgs::PoseWithCovarianceStamped start) {
    //if( m_start_received == false )
        //ROS_INFO("Start received");
    m_start_base = start.pose.pose;
    m_start_received = true;
}
*/

template <class T>
static auto ParseMapFromString(const std::string& s)
    -> std::unordered_map<std::string, T>
{
    std::unordered_map<std::string, T> map;
    std::istringstream ss(s);
    std::string key;
    T value;
    while (ss >> key >> value) {
        map.insert(std::make_pair(key, value));
    }
    return map;
}

bool IsMultiDOFJointVariable(
    const std::string& name,
    std::string* joint_name,
    std::string* local_name)
{
    auto slash_pos = name.find_last_of('/');
    if (slash_pos != std::string::npos) {
        if (joint_name != NULL) {
            *joint_name = name.substr(0, slash_pos);
        }
        if (local_name != NULL) {
            *local_name = name.substr(slash_pos + 1);
        }
        return true;
    } else {
        return false;
    }
}

std::vector<double> getResolutions(
        smpl::RobotModel* robot,
        const PlannerConfig& params ){

    auto resolutions = std::vector<double>(robot->jointVariableCount());

    std::string disc_string = params.discretization;
    if (disc_string.empty()) {
        ROS_ERROR("Parameter 'discretization' not found in planning params");
    }

    auto disc = ParseMapFromString<double>(disc_string);

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        auto& vname = robot->getPlanningJoints()[vidx];
        std::string joint_name, local_name;
        if (IsMultiDOFJointVariable(vname, &joint_name, &local_name)) {
            // adjust variable name if a variable of a multi-dof joint
            auto mdof_vname = joint_name + "_" + local_name;
            auto dit = disc.find(mdof_vname);
            if (dit == end(disc)) {
                ROS_ERROR("Discretization for variable '%s' not found in planning parameters", vname.c_str());
            }
            resolutions[vidx] = dit->second;
        } else {
            auto dit = disc.find(vname);
            if (dit == end(disc)) {
                ROS_ERROR("Discretization for variable '%s' not found in planning parameters", vname.c_str());
            }
            resolutions[vidx] = dit->second;
        }

        ROS_ERROR("resolution(%s) = %0.3f", vname.c_str(), resolutions[vidx]);
    }

    return resolutions;
}

/*
void Callbacks::publish_path(
        std::vector<smpl::RobotState> path,
        ros::Publisher path_pub ){
    ROS_ERROR("publishing path");
    int number_of_joints = 16;
    ros::Rate r(10);

    for(int k=0; k<10000;k++){
    walker_planner::Path1 Final_path;
    for(int i=0;i<path.size();i++){
        walker_planner::GraspPose State;
        for(int j=0;j<path[i].size();j++)
        {
            State.a.push_back(path[i][j]);
        }
        Final_path.path.push_back(State);
    }
    path_pub.publish(Final_path);
    }
}
*/
