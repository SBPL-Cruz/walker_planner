#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <memory>

#include <ros/ros.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/manip_lattice_multi_rep.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/bfs_heuristic_rot.h>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/euclid_diffdrive_heuristic.h>
#include <smpl/heuristic/arm_retract_heuristic.h>
#include <smpl/heuristic/base_rot_euclidean_heuristic.h>
#include <smpl/heuristic/base_rot_bfs_heuristic.h>
#include <sbpl/planners/mrmhaplanner.h>
#include "motion_planner.h"
#include "motion_planner_ros.h"

bool constructHeuristics(
        std::vector<std::unique_ptr<smpl::RobotHeuristic>>& heurs,
        std::shared_ptr<smpl::ManipLatticeMultiRep> pspace,
        smpl::OccupancyGrid& grid,
        std::unique_ptr<smpl::KDLRobotModel>& rm,
        PlannerConfig& params ){

    SMPL_INFO("Initialize Heuristics");

    //Compute a feasible base location.
    std::vector<int> base_x, base_y;

    heurs.clear();
    {
        auto h = std::make_unique<smpl::BfsHeuristic>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        heurs.push_back(std::move(h));
    }
    //{
    //    auto h = std::make_unique<smpl::BfsFullbodyHeuristic>();
    //    h->setCostPerCell(params.cost_per_cell);
    //    h->setInflationRadius(params.inflation_radius);
    //    if (!h->init(pspace.get(), &grid)) {
    //        ROS_ERROR("Could not initialize heuristic.");
    //        return false;
    //    }
    //    SV_SHOW_INFO(h->get2DMapVisualization());
    //    heurs.push_back(std::move(h));
    //}

    {
        auto h = std::make_unique<smpl::BfsHeuristicRot>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        heurs.push_back(std::move(h));
    }
    {
        auto h = std::make_unique<smpl::BfsFullbodyHeuristic>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        SV_SHOW_INFO(h->get2DMapVisualization());
        heurs.push_back(std::move(h));
    }
    for (auto& entry : heurs) {
        pspace->insertHeuristic(entry.get());
    }
    return true;
}

class ReadExperimentFromFile {
    public:

    ReadExperimentFromFile(ros::NodeHandle);
    moveit_msgs::RobotState getStart(PlanningEpisode);
    smpl::GoalConstraint getGoal(PlanningEpisode);

    private:

    ros::NodeHandle m_nh;
    std::vector<moveit_msgs::RobotState> m_start_states;
    std::vector<smpl::GoalConstraint> m_goal_states;

};

ReadExperimentFromFile::ReadExperimentFromFile(ros::NodeHandle _nh) : m_nh{_nh}{
    moveit_msgs::RobotState start_state;
    if (!ReadInitialConfiguration(_nh, start_state)) {
        ROS_ERROR("Failed to get initial configuration.");
    }
    m_start_states.push_back(start_state);
}

moveit_msgs::RobotState ReadExperimentFromFile::getStart(PlanningEpisode _ep){
    return m_start_states.back();
}

smpl::GoalConstraint ReadExperimentFromFile::getGoal(PlanningEpisode _ep){
    return m_goal_states.back();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Rate loop_rate(10);

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
    CollisionSpaceScene scene;

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

    scene.SetCollisionSpace(&cc);

    ROS_INFO("Setting up robot model");
    auto rm = SetupRobotModel(robot_description, robot_config);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    rm->printRobotModelInformation();

    cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

    SV_SHOW_INFO(grid_ptr->getDistanceFieldVisualization(0.2));

    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());

    auto resolutions = getResolutions( rm.get(), planning_config );
    auto multi_action_space = std::make_unique<smpl::ManipLatticeMultiActionSpace>(3);
    auto space = std::make_shared<smpl::ManipLatticeMultiRep>();

    if (!space->init( rm.get(), &cc, resolutions, multi_action_space.get() )) {
        SMPL_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }

    if (!multi_action_space->init(space.get())) {
        SMPL_ERROR("Failed to initialize Manip Lattice Multi Action Space");
        return 1;
    }

    // XXX Loads from filename
    std::vector<std::string> mprim_filenames;
    std::stringstream ss(planning_config.mprim_filenames);
    std::string temp;
    while(getline(ss, temp, ',')){
        mprim_filenames.push_back(temp);
        ROS_ERROR("mprim_filename: %s", temp.c_str());
    }
    for( int i=0; i<3; i++ )
        if(!multi_action_space->load(i, mprim_filenames[i]))
            return 1;

    space->setVisualizationFrameId(grid_ptr->getReferenceFrame());

    using MotionPrimitive = smpl::MotionPrimitive;
    multi_action_space->useMultipleIkSolutions(planning_config.use_multiple_ik_solutions);
    multi_action_space->useAmp(MotionPrimitive::SNAP_TO_XYZ, planning_config.use_xyz_snap_mprim);
    multi_action_space->useAmp(MotionPrimitive::SNAP_TO_RPY, planning_config.use_rpy_snap_mprim);
    multi_action_space->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.use_xyzrpy_snap_mprim);
    multi_action_space->useAmp(MotionPrimitive::SHORT_DISTANCE, planning_config.use_short_dist_mprims);
    multi_action_space->ampThresh(MotionPrimitive::SNAP_TO_XYZ, planning_config.xyz_snap_dist_thresh);
    multi_action_space->ampThresh(MotionPrimitive::SNAP_TO_RPY, planning_config.rpy_snap_dist_thresh);
    multi_action_space->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.xyzrpy_snap_dist_thresh);
    multi_action_space->ampThresh(MotionPrimitive::SHORT_DISTANCE, planning_config.short_dist_mprims_thresh);

    ///////////////
    //Planning////
    /////////////


    std::vector<std::unique_ptr<smpl::RobotHeuristic>> heurs;

    if(!constructHeuristics( heurs, space, *grid_ptr, rm, planning_config )){
        ROS_ERROR("Could not construct heuristics.");
        return 0;
    }

    std::vector<std::unique_ptr<smpl::RobotHeuristic>> inad_heurs;
    for(auto it = heurs.begin() + 1; it != heurs.end(); it++)
        inad_heurs.push_back(std::move(*it));

    //ROS_INFO("Constructing Planner");
    //std::vector<Heuristic*> heur_ptrs;
    //int n_heurs = heurs.size();
    //for(int i=1; i<n_heurs; i++)
    //    heur_ptrs.push_back(heurs[i].get());
    //ROS_INFO("Number of inadmissible heuristics: %d", n_heurs-1);

    //Heuristic** inad = heur_ptrs.data();

    using MotionPlanner = MPlanner::MotionPlanner<MRMHAPlanner, smpl::ManipLatticeMultiRep>;
    MPlanner::PlannerParams planner_params = { 10, 5, 2, false };

    auto mplanner = std::make_unique<MotionPlanner>();
    mplanner->init(std::move(space), std::move(heurs[0]), std::move(inad_heurs), planner_params );

    MotionPlannerROS< Callbacks, ReadExperimentFromFile, MotionPlanner > mplanner_ros(nh);
    mplanner_ros.setPlannerParams(planner_params);

    while(ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
    }
}


