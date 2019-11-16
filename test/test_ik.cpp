#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <memory>

#include <ros/ros.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/manip_lattice_multi_rep.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/euclid_diffdrive_heuristic.h>
#include <smpl/heuristic/arm_retract_heuristic.h>
#include <smpl/heuristic/base_rot_euclidean_heuristic.h>
#include <smpl/heuristic/base_rot_bfs_heuristic.h>
#include <trac_ik_robot_model/trac_ik_robot_model.h>
#include <smpl/debug/marker_utils.h>
#include <sbpl/planners/mrmhaplanner.h>
#include "motion_planner.h"
#include "motion_planner_ros.h"

#define PI 3.14

//using RobotModel = smpl::KDLRobotModel;
using RobotModel = smpl::TracIKRobotModel;

class ReadExperimentFromFile {
    public:

    ReadExperimentFromFile(ros::NodeHandle);
    moveit_msgs::RobotState getStart(PlanningEpisode);
    smpl::GoalConstraint getGoal(PlanningEpisode);

    private:

    ros::NodeHandle m_nh;
    std::vector<moveit_msgs::RobotState> m_start_states;
    std::vector<smpl::GoalConstraint> m_goal_constraints;

};

ReadExperimentFromFile::ReadExperimentFromFile(ros::NodeHandle _nh) : m_nh{_nh}{
    moveit_msgs::RobotState start_state;
    if (!ReadInitialConfiguration(_nh, start_state)) {
        ROS_ERROR("Failed to get initial configuration.");
    }
    m_start_states.push_back(start_state);

    smpl::Affine3 goal_pose;

    std::vector<double> goal_state( 6, 0 );
    m_nh.param("goal/x", goal_state[0], 0.0);
    m_nh.param("goal/y", goal_state[1], 0.0);
    m_nh.param("goal/z", goal_state[2], 0.0);
    m_nh.param("goal/roll", goal_state[3], 0.0);
    m_nh.param("goal/pitch", goal_state[4], 0.0);
    m_nh.param("goal/yaw", goal_state[5], 0.0);

    geometry_msgs::Pose read_grasp;
    read_grasp.position.x = goal_state[0];
    read_grasp.position.y = goal_state[1];
    read_grasp.position.z = goal_state[2];
    tf::Quaternion q;
    q.setRPY( goal_state[3], goal_state[4], goal_state[5] );
    read_grasp.orientation.x = q[0];
    read_grasp.orientation.y = q[1];
    read_grasp.orientation.z = q[2];
    read_grasp.orientation.w = q[3];
    tf::poseMsgToEigen( read_grasp, goal_pose);

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::XYZ_RPY_GOAL;
    goal.pose = goal_pose;
    goal.xyz_tolerance[0] = 0.03;
    goal.xyz_tolerance[1] = 0.03;
    goal.xyz_tolerance[2] = 0.03;
    goal.rpy_tolerance[0] = 0.20;
    goal.rpy_tolerance[1] = 0.20;
    goal.rpy_tolerance[2] = 0.20;

    m_goal_constraints.push_back(goal);
}

moveit_msgs::RobotState ReadExperimentFromFile::getStart(PlanningEpisode _ep){
    return m_start_states.back();
}

smpl::GoalConstraint ReadExperimentFromFile::getGoal(PlanningEpisode _ep){
    return m_goal_constraints.back();
}

class ReadExperimentsFromFile {
    public:
    ReadExperimentsFromFile(ros::NodeHandle);
    moveit_msgs::RobotState getStart(PlanningEpisode);
    smpl::GoalConstraint getGoal(PlanningEpisode);
    bool canCallPlanner() const { return true; }

    private:
    ros::NodeHandle m_nh;
    std::vector<moveit_msgs::RobotState> m_start_states;
    std::vector<smpl::GoalConstraint> m_goal_constraints;

    moveit_msgs::RobotState stringToRobotState(std::string, std::string);
    bool init( std::string, std::string );
};

ReadExperimentsFromFile::ReadExperimentsFromFile(ros::NodeHandle _nh) : m_nh{_nh}{
    std::string start_file, goal_file;
    m_nh.getParam("robot_start_states_file", start_file);
    m_nh.getParam("robot_goal_states_file", goal_file);
    init(start_file, goal_file);
}

moveit_msgs::RobotState ReadExperimentsFromFile::stringToRobotState(
        std::string _header, std::string _state_str){
    moveit_msgs::RobotState robot_state;
    // Assume space separated items.
    std::stringstream header_stream(_header);
    std::stringstream state_stream(_state_str);

    std::string joint_name, joint_val;
    while (header_stream >> joint_name){
        robot_state.joint_state.name.push_back(joint_name);
    }
    while(state_stream >> joint_val){
        robot_state.joint_state.position.push_back(std::stod(joint_val));
    }

    assert(robot_state.joint_state.position.size() == robot_state.joint_state.name.size());
    return robot_state;
}

smpl::GoalConstraint stringToGoalConstraint(std::string _pose_str){

    std::stringstream state_stream(_pose_str);

    std::string pose_val;
    std::vector<double> goal_state( 6, 0 );

    state_stream >> pose_val;
    goal_state[0] = std::stod(pose_val);
    state_stream >> pose_val;
    goal_state[1] = std::stod(pose_val);
    state_stream >> pose_val;
    goal_state[2] = std::stod(pose_val);
    state_stream >> pose_val;
    goal_state[3] = std::stod(pose_val);
    state_stream >> pose_val;
    goal_state[4] = std::stod(pose_val);
    state_stream >> pose_val;
    goal_state[5] = std::stod(pose_val);

    geometry_msgs::Pose read_grasp;
    read_grasp.position.x = goal_state[0];
    read_grasp.position.y = goal_state[1];
    read_grasp.position.z = goal_state[2];

    tf::Quaternion q;
    q.setRPY( goal_state[3], goal_state[4], goal_state[5] );
    read_grasp.orientation.x = q[0];
    read_grasp.orientation.y = q[1];
    read_grasp.orientation.z = q[2];
    read_grasp.orientation.w = q[3];

    smpl::Affine3 goal_pose;
    tf::poseMsgToEigen( read_grasp, goal_pose);

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::XYZ_RPY_GOAL;
    goal.pose = goal_pose;
    goal.xyz_tolerance[0] = 0.05;
    goal.xyz_tolerance[1] = 0.05;
    goal.xyz_tolerance[2] = 0.05;
    goal.rpy_tolerance[0] = 0.39;
    goal.rpy_tolerance[1] = 0.39;
    goal.rpy_tolerance[2] = 0.39;

    return goal;
}

bool ReadExperimentsFromFile::init( std::string _start_file, std::string _goal_file ){
    //Read names of joints from the header.
    std::ifstream start_stream;
    start_stream.open(_start_file);
    ROS_ERROR("%s", _start_file.c_str());
    if(!start_stream)
        throw "Could not read Start file.";
    //std::string header;
    char header[100];
    start_stream.getline(header, 100, '\n');
    std::string header_str(header);

    std::string line_str;
    char line[100];
    while(start_stream.getline(line, 100, '\n')){
        std::string line_str(line);
        m_start_states.push_back(stringToRobotState( header_str, line_str ));
    }

    start_stream.close();

    std::ifstream goal_stream;
    goal_stream.open(_goal_file);
    ROS_ERROR("%s", _goal_file.c_str());
    if(!goal_stream)
        throw "Could not read Goal file.";

    char line2[100];
    while(goal_stream.getline(line2, 100, '\n')){
        std::string line_str(line2);
        m_goal_constraints.push_back(stringToGoalConstraint(line_str));
    }

    goal_stream.close();

    return true;
}

moveit_msgs::RobotState ReadExperimentsFromFile::getStart(PlanningEpisode _ep){
    return m_start_states[_ep];
}

smpl::GoalConstraint ReadExperimentsFromFile::getGoal(PlanningEpisode _ep){
    return m_goal_constraints[_ep];
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ik");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Rate loop_rate(10);
    smpl::VisualizerROS visualizer(nh, 100);
    smpl::viz::set_visualizer(&visualizer);

    /////////////////
    // Robot Model //
    /////////////////

    ROS_INFO("Load common parameters");
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

    // Reads planning_joints, frames.
    RobotModelConfig robot_config;
    if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model"), robot_config)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return 1;
    }

    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    ROS_INFO("Initialize Occupancy Grid");

    auto df_size_x = 20.0;
    auto df_size_y = 15.0;
    auto df_size_z = 1.5;
    auto df_res = 0.05;
    auto df_origin_x = 0;
    auto df_origin_y = 0;
    auto df_origin_z = 0.0;
    auto max_distance = 1.8;

    using DistanceMapType = smpl::EuclidDistanceMap;

    auto df = std::make_shared<DistanceMapType>(
            df_origin_x, df_origin_y, df_origin_z,
            df_size_x, df_size_y, df_size_z,
            df_res,
            max_distance);

    auto ref_counted = false;
    auto grid_ptr = std::make_shared<smpl::OccupancyGrid>(df, ref_counted);

    grid_ptr->setReferenceFrame(planning_frame);
    SV_SHOW_INFO(grid_ptr->getBoundingBoxVisualization());

    //////////////////////////////////
    // Initialize Collision Checker //
    //////////////////////////////////

    ROS_INFO("Initialize collision checker");

    // This whole manage storage for all the scene objects and must outlive
    // its associated CollisionSpace instance.
    auto scene_ptr = std::make_unique<CollisionSpaceScene>();

    smpl::collision::CollisionModelConfig cc_conf;
    if (!smpl::collision::CollisionModelConfig::Load(ph, cc_conf)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return 1;
    }

    ROS_INFO("collision model loaded");
    smpl::collision::CollisionSpace cc;
    if (!cc.init(
            grid_ptr.get(),
            robot_description,
            cc_conf,
            robot_config.group_name,
            robot_config.planning_joints))
    {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }
    cc.setPadding(0.02);
    cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

    //auto marker = cc.getCollisionRobotVisualization(start_state.joint_state.position);
    //marker[0].ns = "Start state";
    //SV_SHOW_INFO(marker);

    /////////////////
    // Setup Scene //
    /////////////////

    PlannerConfig planning_config;
    if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)){
        ROS_ERROR("Failed to read planner config");
        return 1;
    }
    planning_config.cost_per_cell = 1000;
    ROS_INFO("Initialize scene");

    scene_ptr->SetCollisionSpace(&cc);

    auto map_config = getMultiRoomMapConfig(nh);
    std::vector<moveit_msgs::CollisionObject> tmp;
    auto objects = GetMultiRoomMapCollisionCubes(planning_frame, map_config, tmp);
    for (auto& object : objects) {
        scene_ptr->ProcessCollisionObjectMsg(object);
    }

    ROS_INFO("Setting up robot model");
    auto full_rm = SetupRobotModel<RobotModel>(robot_description, robot_config);
    if (!full_rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    robot_config.kinematics_frame = "base_link";
    auto arm_rm = SetupRobotModel<RobotModel>(robot_description, robot_config);
    if (!arm_rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    arm_rm->printRobotModelInformation();

    SV_SHOW_INFO(grid_ptr->getDistanceFieldVisualization(0.2));

    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());

    ros::Duration(1.0).sleep();

    //ReadExperimentFromFile read_exp(ph);
    ReadExperimentsFromFile read_exp(ph);

    int fullbody_ik_successes = 0;
    int arm_ik_successes = 0;
    int N = 10;
    for(int i = 0; i < N; i++){
        auto _start = read_exp.getStart(i);
        smpl::RobotState initial_positions;
        std::vector<std::string> missing;
        if (!leatherman::getJointPositions(
                _start.joint_state,
                _start.multi_dof_joint_state,
                full_rm->getPlanningJoints(),
                initial_positions,
                missing)){

            moveit_msgs::RobotState fixed_state = _start;
            for (auto& variable : missing) {
                fixed_state.joint_state.name.push_back(variable);
                fixed_state.joint_state.position.push_back(0.0);
            }

            if (!leatherman::getJointPositions(
                    fixed_state.joint_state,
                    fixed_state.multi_dof_joint_state,
                    full_rm->getPlanningJoints(),
                    initial_positions,
                    missing)){
                return false;
            }
        }

        auto end_eff_pose = full_rm->computeFK(initial_positions);

        SV_SHOW_INFO_NAMED("fk_robot", cc.getCollisionRobotVisualization(initial_positions));
        SV_SHOW_INFO_NAMED("fk_pose", smpl::visual::MakePoseMarkers(end_eff_pose, planning_frame, "fk_pose"));
        ROS_INFO("Visualizing FK");

        ROS_WARN("Computing Fullbody IK now");

        smpl::RobotState seed_state(initial_positions.size(), 0);
        smpl::RobotState arm_seed_state(initial_positions.size() - 3, 0);
        smpl::RobotState ik_soltn;
        auto success = full_rm->computeIK(end_eff_pose, seed_state, ik_soltn);

        if(success){
            ROS_INFO("IK Succeeded");
            fullbody_ik_successes++;
            auto markers = cc.getCollisionRobotVisualization(ik_soltn);
            int idx = 0;
            for (auto& m : markers.markers) {
                m.ns = "ik_solution";
                m.id = idx;
                idx++;
            }
            visualizer.visualize(smpl::visual::Level::Info, markers);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } else {
            ROS_ERROR("IK Failed.");
        }

        //ros::Duration(2.0).sleep();

        ROS_WARN("Computing Arm IK now");

        ik_soltn.clear();
        //success = arm_rm->computeIK(end_eff_pose, arm_seed_state, ik_soltn);

        if(success){
            ROS_INFO("IK Succeeded");
            arm_ik_successes++;
            auto markers = cc.getCollisionRobotVisualization(ik_soltn);
            int idx = 0;
            for (auto& m : markers.markers) {
                m.ns = "ik_solution";
                m.id = idx;
                idx++;
            }
            visualizer.visualize(smpl::visual::Level::Info, markers);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } else {
            ROS_ERROR("IK Failed.");
        }
        //ros::Duration(2.0).sleep();
    }

    ROS_INFO("IK Stats:");
    ROS_INFO("Fullbody: %d", fullbody_ik_successes);
    ROS_INFO("Arm: %d", arm_ik_successes);

    /*auto goal = read_exp.getGoal(0);

    auto goal_pose = goal.pose;
    double goal_x = goal_pose.translation()[0];
    double goal_y = goal_pose.translation()[1];

    auto rot = goal_pose.rotation();

    //ROS_ERROR("%f, %f, %f, %f", rot.x(), rot.y(), rot.z(), rot.w());
    //tf::Quaternion q { rot.x(), rot.y(), rot.z(), rot.w() };
    //tf::Matrix3x3 m(q);
    double r, p, ya;
    smpl::angles::get_euler_zyx(rot, ya, p, r);
    //m.getRPY(r, p, ya);
    ROS_ERROR("Yaw: %f", ya);

    double base_x=0, base_y=0;
    double arm_length = 0.60;
    double delta = 0;
    double increment = 0.005;

    bool found_base = false;

    std::vector<std::vector<double>> heuristic_base_poses;
    double theta_opt = PI/2 + ya;
    double theta = theta_opt;
    for (int i=0; i<1000; i++) {
        delta += increment;
        // Explore symmetrically about the optimal theta.
        if(i%2)
            theta = theta_opt + delta;
        else
            theta = theta_opt - delta;
        double possible_x = goal_x + arm_length*cos(theta);
        double possible_y = goal_y + arm_length*sin(theta);
        double possible_yaw = atan2(goal_y - possible_y, goal_x - possible_x);
        smpl::RobotState possible_state(full_rm->jointCount(), 0);
        possible_state[0] = possible_x;
        possible_state[1] = possible_y;
        possible_state[2] = possible_yaw;

        if (cc.isStateValid(possible_state)) {
            heuristic_base_poses.push_back(possible_state);
            found_base = true;
            ROS_INFO("Success on Iteration: %d", i);
            ROS_INFO("Optimal yaw: %f, Found yaw: %f", theta_opt, theta);
            break;
        }
    }
    if (!found_base) {
        ROS_ERROR("Could not find a valid base state.");
    }
    else {
        ROS_ERROR( "%d Valid base pose found.", heuristic_base_poses.size() );

        smpl::RobotState possible_base(seed_state.size(), 0);
        possible_base[0] = heuristic_base_poses[0][0];
        possible_base[1] = heuristic_base_poses[0][1];
        possible_base[2] = heuristic_base_poses[0][2];
        auto markers = cc.getCollisionRobotVisualization(possible_base);
        int idx = 0;
        for (auto& m : markers.markers) {
            m.ns = "possible_base";
            m.id = idx;
            idx++;
        }
        visualizer.visualize(smpl::visual::Level::Info, markers);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    SV_SHOW_INFO_NAMED("goal_pose", smpl::visual::MakePoseMarkers(goal.pose, planning_frame, "goal_pose"));

    seed_state[0] = heuristic_base_poses[0][0];
    seed_state[1] = heuristic_base_poses[0][1];
    seed_state[2] = heuristic_base_poses[0][2];
    ROS_ERROR("Size: %d", seed_state.size());
    auto success = full_rm->computeIK(goal.pose, seed_state, ik_soltn);

    if(success){
        ROS_INFO("IK Succeeded");
        auto markers = cc.getCollisionRobotVisualization(ik_soltn);
        int idx = 0;
        for (auto& m : markers.markers) {
            m.ns = "ik_solution";
            m.id = idx;
            idx++;
        }
        visualizer.visualize(smpl::visual::Level::Info, markers);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } else {
        ROS_ERROR("IK Failed.");
    }*/
}


