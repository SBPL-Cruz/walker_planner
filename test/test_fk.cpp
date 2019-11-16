#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <memory>
#include <fstream>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/manip_lattice_multi_rep.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/heuristic/bfs_base_rot_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/euclid_diffdrive_heuristic.h>
#include <smpl/heuristic/arm_retract_heuristic.h>
#include <smpl/types.h>
#include <sbpl/planners/mrmhaplanner.h>
#include "motion_planner.h"
#include "motion_planner_ros.h"

void printPose(const Eigen::Affine3d& pose){
    auto trans = pose.translation();
    auto rot = pose.rotation();
    double y, p, r;
    smpl::angles::get_euler_zyx(rot, y, p, r);

    SMPL_WARN("x: %f, y: %f, z: %f", trans.x(), trans.y(), trans.z());
    SMPL_WARN("r: %f, p: %f, y: %f", r, p, y);
}

double computeDistance(
        const smpl::Vector3 a,
        const smpl::Vector3 b ){

    auto sqrd = [](double d) { return d * d; };

    auto diff = b - a;
    double dist = std::sqrt(
            sqrd(diff.x()) +
            sqrd(diff.y()) +
            sqrd(diff.z())
            );
    return dist;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_fk");
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
    auto grid_ptr = std::make_unique<smpl::OccupancyGrid>(df, ref_counted);

    grid_ptr->setReferenceFrame(planning_frame);
    SV_SHOW_INFO(grid_ptr->getBoundingBoxVisualization());

    //////////////////////////////////
    // Initialize Collision Checker //
    //////////////////////////////////

    ROS_INFO("Initialize collision checker");

    // This whole manage storage for all the scene objects and must outlive
    // its associated CollisionSpace instance.
    //auto scene_ptr = std::make_unique<CollisionSpaceScene>();
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

    ROS_INFO("Setting up robot model");
    auto rm = SetupRobotModel<smpl::KDLRobotModel>(robot_description, robot_config);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    rm->printRobotModelInformation();


    SV_SHOW_INFO(grid_ptr->getDistanceFieldVisualization(0.2));

    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());

    ros::Duration(1.0).sleep();

    auto resolutions = getResolutions( rm.get(), planning_config );
    auto action_space = std::make_unique<smpl::ManipLatticeActionSpace>();
    auto space = std::make_unique<smpl::ManipLattice>();

    if (!space->init( rm.get(), &cc, resolutions, action_space.get() )) {
        SMPL_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }

    if (!action_space->init(space.get())) {
        SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
        return 1;
    }

    if(!action_space->load(planning_config.mprim_filename))
        return 1;

    space->setVisualizationFrameId(grid_ptr->getReferenceFrame());

    using MotionPrimitive = smpl::MotionPrimitive;
    action_space->useMultipleIkSolutions(planning_config.use_multiple_ik_solutions);
    action_space->useAmp(MotionPrimitive::SNAP_TO_XYZ, planning_config.use_xyz_snap_mprim);
    action_space->useAmp(MotionPrimitive::SNAP_TO_RPY, planning_config.use_rpy_snap_mprim);
    action_space->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.use_xyzrpy_snap_mprim);
    action_space->useAmp(MotionPrimitive::SHORT_DISTANCE, planning_config.use_short_dist_mprims);
    action_space->ampThresh(MotionPrimitive::SNAP_TO_XYZ, planning_config.xyz_snap_dist_thresh);
    action_space->ampThresh(MotionPrimitive::SNAP_TO_RPY, planning_config.rpy_snap_dist_thresh);
    action_space->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.xyzrpy_snap_dist_thresh);
    action_space->ampThresh(MotionPrimitive::SHORT_DISTANCE, planning_config.short_dist_mprims_thresh);

    moveit_msgs::RobotState start_state;
    if (!ReadInitialConfiguration(ph, start_state)) {
        ROS_ERROR("Failed to get initial configuration.");
    }

    smpl::RobotState initial_positions;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            start_state.joint_state,
            start_state.multi_dof_joint_state,
            rm->getPlanningJoints(),
            initial_positions,
            missing)){
        ROS_WARN_STREAM("start state is missing planning joints: " << missing);

        moveit_msgs::RobotState fixed_state = start_state;
        for (auto& variable : missing) {
            ROS_WARN("  Assume position 0.0 for joint variable '%s'", variable.c_str());
            fixed_state.joint_state.name.push_back(variable);
            fixed_state.joint_state.position.push_back(0.0);
        }

        if (!leatherman::getJointPositions(
                fixed_state.joint_state,
                fixed_state.multi_dof_joint_state,
                rm->getPlanningJoints(),
                initial_positions,
                missing)){
            return 1;
        }
    }

    smpl::RobotState home_state( initial_positions.size(), 0 );
    auto home_pose = rm->computeFK(home_state);
    printPose(home_pose);

    std::vector<smpl::Action> actions;
    if (!action_space->apply(home_state, actions)) {
        SMPL_WARN("Failed to get actions");
        return 1;
    }
    std::vector<int> num_cells;
    for(auto& action : actions){
        auto pose = rm->computeFK(action[0]);
        auto dist = computeDistance(home_pose.translation(), pose.translation());
        num_cells.push_back(ceil(dist/df_res));
        //SMPL_WARN("Displacement: %f", dist);
    }
    int max = *std::max_element(num_cells.begin(), num_cells.end());
    ROS_ERROR("Max num of cells traversed: %d", max);
    ROS_ERROR("Max cost per cell for bfs: DefaultCostMultiplier/%d", max);

}
