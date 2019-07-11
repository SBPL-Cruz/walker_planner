////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2012, Benjamin Cohen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Benjamin Cohen

// standard includes
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>
#include <ros/package.h>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <kdl_conversions/kdl_msg.h>
#include <smpl/ros/planner_interface.h>
#include <smpl/heuristic/euclid_fullbody_heuristic.h>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/spatial.h>
#include <smpl/distance_map/edge_euclid_distance_map.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/ros/propagation_distance_field.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <visualization_msgs/MarkerArray.h>
#include <smpl/angles.h>
#include <smpl/debug/visualizer_ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "collision_space_scene.h"
#include <nav_msgs/OccupancyGrid.h>
#include <walker_planner/Path1.h>
#include "publish_path.h"


int plan(ros::NodeHandle nh, ros::NodeHandle ph, geometry_msgs::Pose grasp, octomap_msgs::OctomapWithPose octomap);
void FillGoalConstraint(
    const std::vector<double>& pose,
    std::string frame_id,
    moveit_msgs::Constraints& goals,
    double xyz_tol,
    double rpy_tol)
{
    if (pose.size() < 6) {
        return;
    }

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose[0];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose[1];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose[2];

//    goals.position_constraints[0].position.x = pose[0];
//    goals.position_constraints[0].position.y = pose[1];
//    goals.position_constraints[0].position.z = pose[2];

    Eigen::Quaterniond q;
    smpl::angles::from_euler_zyx(pose[5], pose[4], pose[3], q);
    tf::quaternionEigenToMsg(q, goals.orientation_constraints[0].orientation);

    geometry_msgs::Pose p;
    p.position = goals.position_constraints[0].constraint_region.primitive_poses[0].position;
    p.orientation = goals.orientation_constraints[0].orientation;
    leatherman::printPoseMsg(p, "Goal");

    /// set tolerances
    goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, xyz_tol);
    goals.orientation_constraints[0].absolute_x_axis_tolerance = rpy_tol;
    goals.orientation_constraints[0].absolute_y_axis_tolerance = rpy_tol;
    goals.orientation_constraints[0].absolute_z_axis_tolerance = rpy_tol;

    ROS_INFO("Done packing the goal constraints message.");
}

auto GetCollisionCube(
    const geometry_msgs::Pose& pose,
    std::vector<double>& dims,
    const std::string& frame_id,
    const std::string& id)
    -> moveit_msgs::CollisionObject
{
    moveit_msgs::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box_object;
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

auto GetCollisionCubes(
    std::vector<std::vector<double>>& objects,
    std::vector<std::string>& object_ids,
    const std::string& frame_id)
    -> std::vector<moveit_msgs::CollisionObject>
{
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3,0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        ROS_INFO("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(GetCollisionCube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

auto GetCollisionObjects(
    const std::string& filename,
    const std::string& frame_id)
    -> std::vector<moveit_msgs::CollisionObject>
{
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double> > objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    FILE* fCfg = fopen(filename.c_str(), "r");

    if (fCfg == NULL) {
        ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg,"%s",sTemp) < 1) {
        printf("Parsed string has length < 1.\n");
    }

    num_obs = atoi(sTemp);

    ROS_INFO("%i objects in file",num_obs);

    //get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i=0; i < num_obs; ++i) {
        if (fscanf(fCfg,"%s",sTemp) < 1) {
            printf("Parsed string has length < 1.\n");
        }
        object_ids.push_back(sTemp);

        objects[i].resize(6);
        for (int j=0; j < 6; ++j)
        {
            if (fscanf(fCfg,"%s",sTemp) < 1) {
                printf("Parsed string has length < 1.\n");
            }
            if (!feof(fCfg) && strlen(sTemp) != 0) {
                objects[i][j] = atof(sTemp);
            }
        }
    }

    return GetCollisionCubes(objects, object_ids, frame_id);
}

bool ReadInitialConfiguration(
    ros::NodeHandle& nh,
    moveit_msgs::RobotState& state)
{
    XmlRpc::XmlRpcValue xlist;

    // joint_state
    if (nh.hasParam("initial_configuration/joint_state")) {
        ros::param::get("initial_configuration/joint_state", xlist);

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("initial_configuration/joint_state is not an array.");
        }

        if (xlist.size() > 0) {
            for (int i = 0; i < xlist.size(); ++i) {
                state.joint_state.name.push_back(std::string(xlist[i]["name"]));

                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    state.joint_state.position.push_back(double(xlist[i]["position"]));
                }
                else {
                    ROS_DEBUG("Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')");
                    if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        int pos = xlist[i]["position"];
                        state.joint_state.position.push_back(double(pos));
                    }
                }
            }
        }
    }
    else {
        ROS_WARN("initial_configuration/joint_state is not on the param server.");
    }

    // multi_dof_joint_state
    if (nh.hasParam("initial_configuration/multi_dof_joint_state")) {
        nh.getParam("initial_configuration/multi_dof_joint_state", xlist);

        if (xlist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if (xlist.size() != 0) {
                auto &multi_dof_joint_state = state.multi_dof_joint_state;
                multi_dof_joint_state.joint_names.resize(xlist.size());
                multi_dof_joint_state.transforms.resize(xlist.size());
                for (int i = 0; i < xlist.size(); ++i) {
                    multi_dof_joint_state.joint_names[i] = std::string(xlist[i]["joint_name"]);

                    Eigen::Quaterniond q;
                    smpl::angles::from_euler_zyx(
                            (double)xlist[i]["yaw"], (double)xlist[i]["pitch"], (double)xlist[i]["roll"], q);

                    geometry_msgs::Quaternion orientation;
                    tf::quaternionEigenToMsg(q, orientation);

                    multi_dof_joint_state.transforms[i].translation.x = xlist[i]["x"];
                    multi_dof_joint_state.transforms[i].translation.y = xlist[i]["y"];
                    multi_dof_joint_state.transforms[i].translation.z = xlist[i]["z"];
                    multi_dof_joint_state.transforms[i].rotation.w = orientation.w;
                    multi_dof_joint_state.transforms[i].rotation.x = orientation.x;
                    multi_dof_joint_state.transforms[i].rotation.y = orientation.y;
                    multi_dof_joint_state.transforms[i].rotation.z = orientation.z;
                }
            } else {
                ROS_WARN("initial_configuration/multi_dof_joint_state array is empty");
            }
        } else {
            ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");
        }
    }

    ROS_INFO("Read initial state containing %zu joints and %zu multi-dof joints", state.joint_state.name.size(), state.multi_dof_joint_state.joint_names.size());
    return true;
}

struct RobotModelConfig
{
    std::string group_name;
    std::vector<std::string> planning_joints;
    std::string kinematics_frame;
    std::string chain_tip_link;
    std::string arm_start_link;
};

bool ReadRobotModelConfig(const ros::NodeHandle &nh, RobotModelConfig &config)
{
    if (!nh.getParam("group_name", config.group_name)) {
        ROS_ERROR("Failed to read 'group_name' from the param server");
        return false;
    }

    std::string planning_joint_list;
    if (!nh.getParam("planning_joints", planning_joint_list)) {
        ROS_ERROR("Failed to read 'planning_joints' from the param server");
        return false;
    }

    std::stringstream joint_name_stream(planning_joint_list);
    while (joint_name_stream.good() && !joint_name_stream.eof()) {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.empty()) {
            continue;
        }
        config.planning_joints.push_back(jname);
    }

    // only required for generic kdl robot model?
    nh.getParam("kinematics_frame", config.kinematics_frame);
    nh.getParam("chain_tip_link", config.chain_tip_link);
    nh.getParam("arm_start_link", config.arm_start_link);
    return true;
}

struct PlannerConfig
{
    std::string discretization;
    std::string mprim_filename;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_dist_thresh;
    double rpy_snap_dist_thresh;
    double xyzrpy_snap_dist_thresh;
    double short_dist_mprims_thresh;
};

bool ReadPlannerConfig(const ros::NodeHandle &nh, PlannerConfig &config)
{
    if (!nh.getParam("discretization", config.discretization)) {
        ROS_ERROR("Failed to read 'discretization' from the param server");
        return false;
    }

    if (!nh.getParam("mprim_filename", config.mprim_filename)) {
        ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyz_snap_mprim", config.use_xyz_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_rpy_snap_mprim", config.use_rpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_rpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyzrpy_snap_mprim", config.use_xyzrpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyzrpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_short_dist_mprims", config.use_short_dist_mprims)) {
        ROS_ERROR("Failed to read param 'use_short_dist_mprims' from the param server");
        return false;
    }

    if (!nh.getParam("xyz_snap_dist_thresh", config.xyz_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyz_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("rpy_snap_dist_thresh", config.rpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'rpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("xyzrpy_snap_dist_thresh", config.xyzrpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyzrpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("short_dist_mprims_thresh", config.short_dist_mprims_thresh)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    return true;
}

auto SetupRobotModel(const std::string& urdf, const RobotModelConfig &config)
    -> std::unique_ptr<smpl::KDLRobotModel>
{
    if (config.kinematics_frame.empty() || config.chain_tip_link.empty()) {
        ROS_ERROR("Failed to retrieve param 'kinematics_frame' or 'chain_tip_link' from the param server");
        return NULL;
    }

    ROS_INFO("Construct Generic KDL Robot Model");
    std::unique_ptr<smpl::KDLRobotModel> rm(new smpl::KDLRobotModel);

    if (!rm->init(urdf, config.kinematics_frame, config.chain_tip_link)) {
        ROS_ERROR("Failed to initialize robot model.");
        return NULL;
    }

    return std::move(rm);
}

auto SetupArmModel(const std::string& urdf, const RobotModelConfig &config)
    -> std::unique_ptr<smpl::KDLRobotModel>
{
    if (config.arm_start_link.empty() || config.chain_tip_link.empty()) {
        ROS_ERROR("Failed to retrieve param 'arm_start_link' or 'chain_tip_link' from the param server");
        return NULL;
    }

    ROS_INFO("Construct Generic KDL Arm Model");
    std::unique_ptr<smpl::KDLRobotModel> rm(new smpl::KDLRobotModel);

    if (!rm->init(urdf, config.arm_start_link, config.chain_tip_link)) {
        ROS_ERROR("Failed to initialize robot model.");
        return NULL;
    }

    return std::move(rm);
}

class MsgSubscriber {
    public:
        MsgSubscriber(ros::NodeHandle nh, ros::NodeHandle ph) {
            m_nh = nh;
            m_ph = ph;
            m_start_received = false;
            m_octomap_received = false;
            m_grasp_received = false;
            m_occgrid_received = false;

            ros::param::set("/walker_planner_request", 0);
            ros::param::set("/walker_planner_done", 0);
            m_path_pub = m_nh.advertise<walker_planner::Path1>("Robot_path", 1000);
            m_sub_start = m_nh.subscribe("/poseupdate", 1000, &MsgSubscriber::startCallback, this);
            m_sub_occgrid = m_nh.subscribe("/map", 1000, &MsgSubscriber::occgridCallback, this);
            m_sub_octomap = m_nh.subscribe("/octomap_binary", 1000, &MsgSubscriber::octomapCallback, this);
            m_sub_pose = m_nh.subscribe("/Grasps", 1000, &MsgSubscriber::poseCallback, this);
        }

        void octomapCallback(const octomap_msgs::Octomap& msg) {
            ROS_ERROR("Octomap received");
            m_map_with_pose.header.frame_id = "map";
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

        void occgridCallback(const nav_msgs::OccupancyGrid& msg) {
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
                          for (int h=0; h < 0.72/res; h++) {
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
                                //if (planning_mode == "BASE") {
                                //    points.push_back(v2);
                                //    points.push_back(v3);
                                //}
                         }
                       // }
                    }
                }
            }
            m_occgrid_points = points;
            m_occgrid_received = true;
            ROS_INFO("Occupancy Grid loaded.");
        }

        void poseCallback(const geometry_msgs::PoseStamped grasp) {
            m_grasp = grasp.pose;
            m_grasp_received = true;

            std::string planning_mode = "FULLBODY";
            ros::param::get("/walker_planner_mode", planning_mode);
	    ROS_ERROR("mode: %s", planning_mode.c_str());
	    ROS_ERROR("start_received: %d", m_start_received);
	    ROS_ERROR("occgrid received: %d", m_occgrid_received);
            if( ( planning_mode == "FULLBODY" ) && m_start_received &&
                   m_occgrid_received &&  m_octomap_received) {
                ROS_ERROR("Fullbody Planner Called.");
                m_octomap_received = false;
                m_start_received = false;
                m_grasp_received = false;
                plan(m_nh, m_ph, m_grasp, m_map_with_pose);
            }
            else if( ( planning_mode == "BASE" ) && m_start_received  &&
                    m_occgrid_received ) {
                ROS_ERROR("Base Planner Called.");
                plan(m_nh, m_ph, m_grasp, m_map_with_pose);
            }
            else{
                ROS_ERROR("Planner mode not understood.");
            }
        }


        void startCallback(const geometry_msgs::PoseWithCovarianceStamped start) {
            if( m_start_received == false )
                ROS_INFO("Start received");
            m_start_base = start.pose.pose;
            m_start_received = true;
        }

        int plan(ros::NodeHandle nh, ros::NodeHandle ph, geometry_msgs::Pose grasp, octomap_msgs::OctomapWithPose octomap);

        octomap_msgs::OctomapWithPose subscribeOctomap(ros::NodeHandle nh) {
            return m_map_with_pose;
        }

        geometry_msgs::Pose subscribeGrasp(ros::NodeHandle nh) {
            return m_grasp;
        }

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
        bool m_octomap_received;
        bool m_grasp_received;
        bool m_start_received;
        bool m_occgrid_received;
};


int MsgSubscriber::plan(ros::NodeHandle nh, ros::NodeHandle ph, geometry_msgs::Pose grasp, octomap_msgs::OctomapWithPose octomap) {
    int req, done;
    ros::param::get("/walker_planner_request", req);
    if (req) {
        ros::param::set("/walker_planner_request", 0);

        ROS_INFO("Initialize visualizer");
        smpl::VisualizerROS visualizer(nh, 100);
        smpl::viz::set_visualizer(&visualizer);

        // Let publishers set up
        ros::Duration(1.0).sleep();

        /////////////////
        // Robot Model //
        /////////////////

        ROS_INFO("Load common parameters");

        // Robot description required to initialize collision checker and robot
        // model...
        auto robot_description_key = "robot_description";
        std::string robot_description_param;
        if (!nh.searchParam(robot_description_key, robot_description_param)) {
            ROS_ERROR("Failed to find 'robot_description' key on the param server");
            return 1;
        }

        std::string robot_description;
        if (!nh.getParam(robot_description_param, robot_description)) {
            ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
            return 1;
        }

        std::string planning_mode = "FULLBODY";
        ros::param::get("/walker_planner_mode", planning_mode);
        if (planning_mode == "BASE") {
            ROS_INFO("Switching to BASE planning mode.");
            ros::param::set("/walker_interface_planner/robot_model/chain_tip_link", "base_link");
            ros::param::set("/walker_interface_planner/robot_model/planning_joints", "x y theta");
            std::string pkg_path = ros::package::getPath("walker_planner");
            ros::param::set("/walker_interface_planner/planning/mprim_filename", pkg_path + "/config/walker_base.mprim");
        }
        else {
          ROS_INFO("Switching to FULLBODY planning mode.");
          ros::param::set("/walker_interface_planner/robot_model/chain_tip_link", "right_palm_link");
          ros::param::set("/walker_interface_planner/robot_model/planning_joints", "x  y theta right_j1  right_j2 right_j3 right_j4 right_j5 right_j6 right_j7" );
          std::string pkg_path = ros::package::getPath("walker_planner");
          ros::param::set("/walker_interface_planner/planning/mprim_filename", pkg_path + "/config/walker_right_arm.mprim");
        }


        // Reads planning_joints, frames.
        RobotModelConfig robot_config;
        if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model"), robot_config)) {
            ROS_ERROR("Failed to read robot model config from param server");
            return 1;
        }

        // Everyone needs to know the name of the planning frame for reasons...
        // ...frame_id for the occupancy grid (for visualization)
        // ...frame_id for collision objects (must be the same as the grid, other than that, useless)
        std::string planning_frame;
        if (!ph.getParam("planning_frame", planning_frame)) {
            ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
            return 1;
        }
        // Read in start state from file and update the scene...
        // Start state is also required by the planner...
        moveit_msgs::RobotState start_state;
        if (!ReadInitialConfiguration(ph, start_state)) {
            ROS_ERROR("Failed to get initial configuration.");
            return 1;
        }

        //while(!m_start_received) {
        //    ROS_ERROR("Waiting for the Start state from AMCL.");
        //}
        //ROS_INFO("Start received");
        // m_start_base.position.x = 0;
        // m_start_base.position.y = 0;
        start_state.joint_state.position[1] = m_start_base.position.y;
        start_state.joint_state.position[0] = m_start_base.position.x;

        {
        double roll, pitch, yaw;
        tf::Quaternion q(
            m_start_base.orientation.x,
            m_start_base.orientation.y,
            m_start_base.orientation.z,
            m_start_base.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        start_state.joint_state.position[2] = yaw;
        }


        ////////////////////
        // Occupancy Grid //
        ////////////////////

        ROS_INFO("Initialize Occupancy Grid");

        auto df_size_x = 15.0;
        auto df_size_y = 15.0;
        auto df_size_z = 2.0;
        auto df_res = 0.05;
        auto df_origin_x = m_start_base.position.x - 5;
        auto df_origin_y = m_start_base.position.y - 4;
        auto df_origin_z = 0.0;
        auto max_distance = 1.8;

        using DistanceMapType = smpl::EuclidDistanceMap;

        auto df = std::make_shared<DistanceMapType>(
                df_origin_x, df_origin_y, df_origin_z,
                df_size_x, df_size_y, df_size_z,
                df_res,
                max_distance);

        auto ref_counted = false;
        smpl::OccupancyGrid grid(df, ref_counted);

        //while(m_occgrid_received == false) {
        //    ROS_INFO("Waiting for 2D occupancy grid.");
        //    ros::Duration(1.0).sleep();
        //}
        grid.addPointsToField(m_occgrid_points);
        ROS_ERROR("Occupied voxesls: %d", grid.getOccupiedVoxelCount());
    //////////////////////////////////
    // Initialize Collision Checker //
    //////////////////////////////////


        grid.setReferenceFrame(planning_frame);
        SV_SHOW_INFO(grid.getBoundingBoxVisualization());

        //////////////////////////////////
        // Initialize Collision Checker //
        //////////////////////////////////

        ROS_INFO("Initialize collision checker");

        // This whole manage storage for all the scene objects and must outlive
        // its associated CollisionSpace instance.
        CollisionSpaceScene scene;

        smpl::collision::CollisionModelConfig cc_conf;
        if (!smpl::collision::CollisionModelConfig::Load(ph, cc_conf)) {
            ROS_ERROR("Failed to load Collision Model Config");
            return 1;
        }

        ROS_INFO("collision model loaded");
        smpl::collision::CollisionSpace cc;
        if (!cc.init(
                &grid,
                robot_description,
                cc_conf,
                robot_config.group_name,
                robot_config.planning_joints))
        {
            ROS_ERROR("Failed to initialize Collision Space");
            return 1;
        }
        cc.setPadding(0.02);

        auto marker = cc.getCollisionRobotVisualization(start_state.joint_state.position);
        //marker[0].ns = "Start state";
        SV_SHOW_INFO(marker);
        visualizer.visualize(smpl::visual::Level::Info, marker);


        std::vector<double> goal_state(6, 0);

        {
        ROS_INFO("Taking your goal");
            double roll, pitch, yaw;
            tf::Quaternion q(
                grasp.orientation.x,
                grasp.orientation.y,
                grasp.orientation.z,
                grasp.orientation.w);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            goal_state[0] = grasp.position.x;
            goal_state[1] = grasp.position.y;
            goal_state[2] = grasp.position.z;
            goal_state[3] = roll;
            goal_state[4] = pitch;
            goal_state[5] = yaw;
        }
         bool read_goal_from_file;
         ph.param("read_goal_from_file", read_goal_from_file, true );
         // Read from file.
         if (read_goal_from_file) {
             ph.param("goal/x", goal_state[0], 0.0);
             ph.param("goal/y", goal_state[1], 0.0);
             ph.param("goal/z", goal_state[2], 0.0);
             ph.param("goal/roll", goal_state[3], 0.0);
             ph.param("goal/pitch", goal_state[4], 0.0);
             ph.param("goal/yaw", goal_state[5], 0.0);
             ROS_ERROR("Goal z: %f", goal_state[2]);
         }
        /////////////////
        // Setup Scene //
        /////////////////

        ROS_INFO("Initialize scene");

        scene.SetCollisionSpace(&cc);

        std::string object_filename;
        ph.param<std::string>("object_filename", object_filename, "");

        // Read in collision objects from file and add to the scene...
        if (!object_filename.empty()) {
            auto objects = GetCollisionObjects(object_filename, planning_frame);
            for (auto& object : objects) {
                scene.ProcessCollisionObjectMsg(object);
            }
        }

        ROS_INFO("Setting up robot model");
        auto rm = SetupRobotModel(robot_description, robot_config);
        if (!rm) {
            ROS_ERROR("Failed to set up Robot Model");
            return 1;
        }

        auto arm_rm = SetupArmModel(robot_description, robot_config);
        if(!arm_rm){
            ROS_ERROR("Failed to set up Arm modle.");
            return 1;
        }

        //XXX
        rm->printRobotModelInformation();

        // Set reference state in the robot planning model...
        smpl::urdf::RobotState reference_state;
        InitRobotState(&reference_state, &rm->m_robot_model);
        for (auto i = 0; i < start_state.joint_state.name.size(); ++i) {
            auto* var = GetVariable(&rm->m_robot_model, &start_state.joint_state.name[i]);
            if (var == NULL) {
                ROS_WARN("Failed to do the thing");
                continue;
            }
            ROS_INFO("Set joint %s to %f", start_state.joint_state.name[i].c_str(), start_state.joint_state.position[i]);
            SetVariablePosition(&reference_state, var, start_state.joint_state.position[i]);
        }
        SetReferenceState(rm.get(), GetVariablePositions(&reference_state));

        // Set reference state in the collision model...
        if (!scene.SetRobotState(start_state)) {
            ROS_ERROR("Failed to set start state on Collision Space Scene");
            return 1;
        }

        //while (planning_mode == "FULLBODY" && m_octomap_received == false) {
        //    ROS_ERROR("Waiting for octomap.");
        //    ros::Duration(1.0).sleep();
        //}
        if( planning_mode == "FULLBODY" ){
            bool ret = scene.ProcessOctomapMsg(octomap);
            if(ret)
                ROS_INFO("Succesfully added ocotmap");
            else
                ROS_INFO("Could not added ocotmap");
        }

        cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

        SV_SHOW_INFO(grid.getDistanceFieldVisualization(0.2));

        SV_SHOW_INFO(cc.getCollisionRobotVisualization());
        SV_SHOW_INFO(cc.getCollisionWorldVisualization());
        SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());
        visualizer.visualize(smpl::visual::Level::Info, cc.getCollisionWorldVisualization());
        visualizer.visualize(smpl::visual::Level::Info, cc.getCollisionRobotVisualization());
        visualizer.visualize(smpl::visual::Level::Info, cc.getOccupiedVoxelsVisualization());

        ///////////////////
        // Planner Setup //
        ///////////////////

        PlannerConfig planning_config;
        if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)) {
            ROS_ERROR("Failed to read planner config");
            return 1;
        }

        smpl::PlannerInterface planner(rm.get(), &cc, &grid);

        smpl::PlanningParams params;
        ROS_ERROR("mprim_filename, %s", planning_config.mprim_filename.c_str());
        params.addParam("discretization", planning_config.discretization);
        params.addParam("mprim_filename", planning_config.mprim_filename);
        params.addParam("use_xyz_snap_mprim", planning_config.use_xyz_snap_mprim);
        params.addParam("use_rpy_snap_mprim", planning_config.use_rpy_snap_mprim);
        params.addParam("use_xyzrpy_snap_mprim", planning_config.use_xyzrpy_snap_mprim);
        params.addParam("use_short_dist_mprims", planning_config.use_short_dist_mprims);
        params.addParam("xyz_snap_dist_thresh", planning_config.xyz_snap_dist_thresh);
        params.addParam("rpy_snap_dist_thresh", planning_config.rpy_snap_dist_thresh);
        params.addParam("xyzrpy_snap_dist_thresh", planning_config.xyzrpy_snap_dist_thresh);
        params.addParam("short_dist_mprims_thresh", planning_config.short_dist_mprims_thresh);
        params.addParam("epsilon", 50.0);
        params.addParam("epsilon_mha", 20);
        params.addParam("search_mode", false);
        params.addParam("allow_partial_solutions", false);
        params.addParam("target_epsilon", 1.0);
        params.addParam("delta_epsilon", 1.0);
        params.addParam("improve_solution", false);
        params.addParam("bound_expansions", true);
        params.addParam("repair_time", 1.0);
        params.addParam("bfs_inflation_radius", 0.05);
        params.addParam("bfs_cost_per_cell", 400);
        params.addParam("x_coeff", 1.0);
        params.addParam("y_coeff", 1.0);
        params.addParam("z_coeff", 1.0);
        params.addParam("rot_coeff", 1.0);

     	if (planning_mode == "BASE"){
          params.interpolate_path = false;
          params.shortcut_path = true;
        }
        else if(planning_mode == "FULLBODY"){
          params.interpolate_path = true;
          params.shortcut_path = false;
        }

  	if (!planner.init(params)) {
            ROS_ERROR("Failed to initialize Planner Interface");
            return 1;
        }


        //////////////
        // Planning //
        //////////////


        moveit_msgs::MotionPlanRequest req;
        moveit_msgs::MotionPlanResponse res;

        ph.param("allowed_planning_time", req.allowed_planning_time, 30.0);
        req.goal_constraints.resize(1);
        if(planning_mode == "BASE")
            FillGoalConstraint(goal_state, planning_frame, req.goal_constraints[0], 0.03, 0.2);
        else
            FillGoalConstraint(goal_state, planning_frame, req.goal_constraints[0], 0.05, 2*3.14);

        req.group_name = robot_config.group_name;
        req.max_acceleration_scaling_factor = 1.0;
        req.max_velocity_scaling_factor = 1.0;
        req.num_planning_attempts = 1;
    //    req.path_constraints;
        if (planning_mode == "BASE")
            req.planner_id = "arastar.euclid_diff.manip";
        else
            req.planner_id = "arastar.magic_arm.manip";
        req.start_state = start_state;
    //    req.trajectory_constraints;
    //    req.workspace_parameters;

        // plan
        ROS_INFO("Calling solve...");
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.robot_state = start_state;
        if (!planner.solve(planning_scene, req, res)) {
            ROS_ERROR("Failed to plan.");

            // XXX
            /*
            auto bfs_fullbody_base_poses = (dynamic_cast<smpl::BfsFullbodyHeuristic*>(planner.m_heuristics.begin()->second.get()))->m_heuristic_base_poses;
            int i = 0;
            //for( auto base_pose : bfs_fullbody_base_poses ){
            auto base_pose = bfs_fullbody_base_poses[0];
                smpl::RobotState state( rm->jointCount(), 0 );
                state[0] = base_pose[0];
                state[1] = base_pose[1];
                state[2] = base_pose[2];
                ROS_ERROR("%d dofs and base is %f, %f, %f", state.size(), state[0], state[1], state[2]);
                auto markers = cc.getCollisionRobotVisualization( state );
                for( auto& marker : markers.markers )
                    marker.ns = "ik_base" + std::to_string(i);
                ROS_INFO("Visualizing base position.");
                SV_SHOW_DEBUG_NAMED( "ik_base", markers );
                i++;
            }*/
            return 1;
        }
        // XXX
        // For BFS Fullbody heuristic.
        /*
        auto bfs_fullbody_base_poses = (dynamic_cast<smpl::BfsFullbodyHeuristic*>(planner.m_heuristics.begin()->second.get()))->m_heuristic_base_poses;
        for( auto base_pose : bfs_fullbody_base_poses ){
            smpl::RobotState state( rm->jointCount(), 0 );
            state[0] = base_pose[0];
            state[1] = base_pose[1];
            state[2] = base_pose[2];
            auto markers = cc.getCollisionRobotVisualization( state );
            for( auto& marker : markers.markers )
                marker.ns = "ik_base";
            ROS_INFO("Visualizing base position.");
            SV_SHOW_INFO( marker );
            visualizer.visualize(smpl::visual::Level::Info, marker);
        }
        */


        //-----------------Publishing path---------------------------
        std::vector<smpl::RobotState> path;
        for(int i=0;i<res.trajectory.joint_trajectory.points.size();i++)
        {
        // ROS_INFO("Path size %d",res.trajectory.joint_trajectory.points.size());
        std::vector<double> path_state;
        for(int j=0;j<res.trajectory.joint_trajectory.points[i].positions.size();j++)
        {
            // ROS_INFO("Every state's size %d",res.trajectory.joint_trajectory.points[i].positions.size());
            path_state.push_back(res.trajectory.joint_trajectory.points[i].positions[j]);
        }
        path.push_back(path_state);
        }
        m_path_cached = path;
        publish_path(path, m_path_pub);
        //----------------------------------------------------------------------------------
        ///////////////////////////////////
        // Visualizations and Statistics //
        ///////////////////////////////////

        auto planning_stats = planner.getPlannerStats();

        ROS_INFO("Planning statistics");
        for (auto& entry : planning_stats) {
            ROS_INFO("    %s: %0.3f", entry.first.c_str(), entry.second);
        }

        ROS_INFO("Animate path");

        size_t pidx = 0;
        //while (ros::ok()) {
            auto& point = res.trajectory.joint_trajectory.points[pidx];
            auto markers = cc.getCollisionRobotVisualization(point.positions);
            for (auto& m : markers.markers) {
                m.ns = "path_animation";
            }
            SV_SHOW_INFO(markers);
            visualizer.visualize(smpl::visual::Level::Info, markers);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            pidx++;
            pidx %= res.trajectory.joint_trajectory.points.size();
        //}
        //SV_SHOW_INFO(markers);
        //visualizer.visualize(smpl::visual::Level::Info, markers);
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
        //pidx++;
        //pidx %= res.trajectory.joint_trajectory.points.size();

        ros::param::set("/walker_planner_done", 1);
        // Reset flags

    }
    else {
        int done;
        ros::param::get("/walker_planner_done", done);
        if (done) {
            publish_path(m_path_cached, m_path_pub);
        }
        else {
          //ROS_ERROR("No cached path found.");
        }
    }
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "walker_interface_planner");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Rate loop_rate(10);

    MsgSubscriber sub(nh, ph);
    while(ros::ok()) {
      loop_rate.sleep();
      ros::spinOnce();
    }


    return 0;
}
