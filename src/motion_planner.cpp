#include <tf2/LinearMath/Quaternion.h>
#include <sstream>

#include <smpl/search/arastar.h>
#include <sbpl/planners/mrmhaplanner.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/bfs_heuristic_rot.h>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/euclid_diffdrive_heuristic.h>
#include <smpl/heuristic/arm_retract_heuristic.h>
#include <smpl/heuristic/base_rot_euclidean_heuristic.h>
#include <smpl/heuristic/base_rot_bfs_heuristic.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/manip_lattice_multi_rep.h>
#include <smpl/utils.h>

#include "motion_planner.h"

using namespace smpl;

bool constructHeuristics(
        std::vector<std::unique_ptr<RobotHeuristic>>& heurs,
        std::unique_ptr<ManipLatticeMultiRep>& pspace,
        smpl::OccupancyGrid& grid,
        std::unique_ptr<smpl::KDLRobotModel>& rm,
        const Eigen::Affine3d& goal,
        PlannerConfig& params ){

    SMPL_INFO("Initialize Heuristics");

    //Compute a feasible base location.
    std::vector<int> base_x, base_y;

    heurs.clear();
    {
        auto h = make_unique<BfsHeuristic>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        heurs.push_back(std::move(h));
    }
    //{
    //    auto h = make_unique<BfsFullbodyHeuristic>();
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
        auto h = make_unique<BfsHeuristicRot>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        heurs.push_back(std::move(h));
    }
    {
        auto h = make_unique<BfsFullbodyHeuristic>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        SV_SHOW_INFO(h->get2DMapVisualization());
        heurs.push_back(std::move(h));
    }
    {
        auto h = make_unique<EuclidDiffHeuristic>();
        if (!h->init(pspace.get())) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        h->setWeightRot(0.01);
        //heurs.push_back(std::move(h));
    }


    {
        auto h = make_unique<EuclidDistHeuristic>();
        if (!h->init(pspace.get())) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        h->setWeightRot(0.01);
        //heurs.push_back(std::move(h));
    }
    /*
    {
        auto h = make_unique<ArmRetractHeuristic>();
        if (!h->init(pspace.get())) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        heurs.push_back(std::move(h));
    }
    */

    {
        auto h = make_unique<BaseRotEuclideanHeuristic>();
        if (!h->init(pspace.get(), 1.57)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        h->setWeightRot(0.01);
        //heurs.push_back(std::move(h));
    }

    {
        auto h = make_unique<BaseRotBfsHeuristic>();
        if (!h->init(pspace.get(), &grid, 1.57)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        //heurs.push_back(std::move(h));
    }
    for (auto& entry : heurs) {
        pspace->insertHeuristic(entry.get());
    }
    return true;
}
/*
void visualizeRadiusAroundGoal(int x0, int y0, int radius) {
    std::vector<int> circle_x;
    std::vector<int> circle_y;
    double res = m_occupancy_grid->getResolution();
    int discrete_radius = m_radius/res;
    getBresenhamCirclePoints(x0, y0, discrete_radius, circle_x, circle_y);

    // geometry_msgs::PolygonStamped circle;

    // circle.header.frame_id = "/map";
    // circle.header.stamp = ros::Time::now();
    std::vector<geometry_msgs::Point> circle_points;

    for (size_t i = 0; i < circle_x.size(); ++i) {
        // Prune the points to display only the ones that are within the
        // threshold
        if (m_grid[circle_x[i]][circle_y[i]] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            geometry_msgs::Point out_pt;
            out_pt.x = circle_x[i]*res;
            out_pt.y = circle_y[i]*res;
            out_pt.z = 0.0;
            circle_points.push_back(out_pt);
        }
    }
    std::stringstream ss;
    ss<< "radius_around_goal";
    Visualizer::pviz->visualizeLine(
        circle_points, ss.str(), x0 + y0, 114, 0.01);
}
*/

void getBresenhamCirclePoints( int x0,
        int y0,
        int radius,
        std::vector<int>& ret_x,
        std::vector<int>& ret_y ){
    int x = 0;
    int y = radius;
    int delta = 2 - 2 * radius;
    int err = 0;
    ret_x.clear();
    ret_y.clear();
    while(y >= 0){
        ret_x.push_back(x0 + x);
        ret_x.push_back(x0 - x);
        ret_x.push_back(x0 + x);
        ret_x.push_back(x0 - x);
        ret_y.push_back(y0 - y);
        ret_y.push_back(y0 - y);
        ret_y.push_back(y0 + y);
        ret_y.push_back(y0 + y);
        err = 2 * (delta + y) - 1;
        if(delta < 0 && err <= 0){
                x = x + 1;
                delta = delta + 2 * x + 1;
                continue;
        }
        err = 2 * (delta - x) - 1;
        if(delta > 0 && err > 0){
                y = y - 1;
                delta = delta + 1 - 2 * y;
                continue;
        }
        x = x + 1;
        delta = delta + 2 * (x - y);
        y = y - 1;
    }
}

int MsgSubscriber::plan_mrmha(
        ros::NodeHandle nh,
        ros::NodeHandle ph,
        moveit_msgs::RobotState start_state,
        geometry_msgs::Pose grasp ){
    int req, done;
    ros::param::get("/walker_planner_request", req);
    if (req) {
        ros::param::set("/walker_planner_request", 0);

        ROS_INFO("Initialize visualizer");
        smpl::VisualizerROS visualizer(nh, 100);
        smpl::viz::set_visualizer(&visualizer);

        // Let publishers set up
        ros::Duration(1.0).sleep();

        // Goal Pose
        smpl::Affine3 goal_pose;
        tf::poseMsgToEigen( grasp, goal_pose );

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
        auto df_origin_x = 0;//m_start_base.position.x - 5;
        auto df_origin_y = 0;//m_start_base.position.y - 4;
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

        grid.addPointsToField(m_occgrid_points);
        ROS_ERROR("Occupied voxesls: %d", grid.getOccupiedVoxelCount());

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

        /////////////////
        // Setup Scene //
        /////////////////

        ROS_INFO("Initialize scene");

        scene.SetCollisionSpace(&cc);

        std::string object_filename;
        ph.getParam("object_filename", object_filename);
        ROS_ERROR("%s", object_filename.c_str());

        auto objects = GetMultiRoomMapCollisionCubes(planning_frame, 20, 15, .8, 1);
        for (auto& object : objects) {
            scene.ProcessCollisionObjectMsg(object);
        }

        ROS_INFO("Setting up robot model");
        auto rm = SetupRobotModel(robot_description, robot_config);
        if (!rm) {
            ROS_ERROR("Failed to set up Robot Model");
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

        bool ret = scene.ProcessOctomapMsg(m_map_with_pose);
        if(ret)
            ROS_INFO("Succesfully added octomap");
        else
            ROS_INFO("Could not added octomap");

        cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

        SV_SHOW_INFO(grid.getDistanceFieldVisualization(0.2));
        SV_SHOW_INFO(cc.getCollisionRobotVisualization());
        SV_SHOW_INFO(cc.getCollisionWorldVisualization());
        SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());

        ///////////////////
        // Planner Setup //
        ///////////////////

        PlannerConfig planning_config;
        if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)){
            ROS_ERROR("Failed to read planner config");
            return 1;
        }
        planning_config.cost_per_cell = 1000;

        auto resolutions = getResolutions( rm.get(), planning_config );
        auto actions_full = make_unique<ManipLatticeActionSpace>();
        auto actions_base = make_unique<ManipLatticeActionSpace>();
        auto actions_arm = make_unique<ManipLatticeActionSpace>();
        auto space = make_unique<smpl::ManipLatticeMultiRep>();

        //XXX Assuming:
        //1. Anchor get full
        //2. Inad1 gets arm
        //3. Inad2 gets base
        std::vector<ActionSpace*> v_actions;
        v_actions.push_back(actions_full.get());
        //v_actions.push_back(actions_full.get());
        v_actions.push_back(actions_arm.get());
        v_actions.push_back(actions_base.get());
        if (!space->init( rm.get(), &cc, resolutions, v_actions )) {
            SMPL_ERROR("Failed to initialize Manip Lattice");
            return 1;
        }

        for(auto& actions : v_actions){
            if (!actions->init(space.get())) {
                SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
                return 1;
            }
        }

        space->setVisualizationFrameId(grid.getReferenceFrame());

        actions_full->useMultipleIkSolutions(planning_config.use_multiple_ik_solutions);
        actions_full->useAmp(MotionPrimitive::SNAP_TO_XYZ, planning_config.use_xyz_snap_mprim);
        actions_full->useAmp(MotionPrimitive::SNAP_TO_RPY, planning_config.use_rpy_snap_mprim);
        actions_full->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.use_xyzrpy_snap_mprim);
        actions_full->useAmp(MotionPrimitive::SHORT_DISTANCE, planning_config.use_short_dist_mprims);
        actions_full->ampThresh(MotionPrimitive::SNAP_TO_XYZ, planning_config.xyz_snap_dist_thresh);
        actions_full->ampThresh(MotionPrimitive::SNAP_TO_RPY, planning_config.rpy_snap_dist_thresh);
        actions_full->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.xyzrpy_snap_dist_thresh);
        actions_full->ampThresh(MotionPrimitive::SHORT_DISTANCE, planning_config.short_dist_mprims_thresh);
        actions_base->useMultipleIkSolutions(planning_config.use_multiple_ik_solutions);
        actions_base->useAmp(MotionPrimitive::SNAP_TO_XYZ, planning_config.use_xyz_snap_mprim);
        actions_base->useAmp(MotionPrimitive::SNAP_TO_RPY, planning_config.use_rpy_snap_mprim);
        actions_base->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.use_xyzrpy_snap_mprim);
        actions_base->useAmp(MotionPrimitive::SHORT_DISTANCE, planning_config.use_short_dist_mprims);
        actions_base->ampThresh(MotionPrimitive::SNAP_TO_XYZ, planning_config.xyz_snap_dist_thresh);
        actions_base->ampThresh(MotionPrimitive::SNAP_TO_RPY, planning_config.rpy_snap_dist_thresh);
        actions_base->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.xyzrpy_snap_dist_thresh);
        actions_base->ampThresh(MotionPrimitive::SHORT_DISTANCE, planning_config.short_dist_mprims_thresh);
        actions_arm->useMultipleIkSolutions(planning_config.use_multiple_ik_solutions);
        actions_arm->useAmp(MotionPrimitive::SNAP_TO_XYZ, planning_config.use_xyz_snap_mprim);
        actions_arm->useAmp(MotionPrimitive::SNAP_TO_RPY, planning_config.use_rpy_snap_mprim);
        actions_arm->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.use_xyzrpy_snap_mprim);
        actions_arm->useAmp(MotionPrimitive::SHORT_DISTANCE, planning_config.use_short_dist_mprims);
        actions_arm->ampThresh(MotionPrimitive::SNAP_TO_XYZ, planning_config.xyz_snap_dist_thresh);
        actions_arm->ampThresh(MotionPrimitive::SNAP_TO_RPY, planning_config.rpy_snap_dist_thresh);
        actions_arm->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.xyzrpy_snap_dist_thresh);
        actions_arm->ampThresh(MotionPrimitive::SHORT_DISTANCE, planning_config.short_dist_mprims_thresh);

        // XXX Loads from filename
        std::vector<std::string> mprim_filenames;
        std::stringstream ss(planning_config.mprim_filenames);
        std::string temp;
        while(getline(ss, temp, ',')){
            mprim_filenames.push_back(temp);
            ROS_ERROR("mprim_filename: %s", temp.c_str());
        }

        if (!actions_full->load(mprim_filenames[0])) {
            return 1;
        }
        if (!actions_base->load(mprim_filenames[1])) {
            return 1;
        }
        if (!actions_arm->load(mprim_filenames[2])) {
            return 1;
        }

        //////////////
        // Planning //
        //////////////


        std::vector<std::unique_ptr<smpl::RobotHeuristic>> heurs;
        if(!constructHeuristics( heurs, space, grid, rm, goal_pose, planning_config )){
            ROS_ERROR("Could not construct heuristics.");
            return 0;
        }

        ROS_INFO("Constructing Planner");
        std::vector<Heuristic*> heur_ptrs;
        int n_heurs = heurs.size();
        for(int i=1; i<n_heurs; i++)
            heur_ptrs.push_back(heurs[i].get());
        ROS_INFO("Number of inadmissible heuristics: %d", n_heurs-1);

        Heuristic** inad = heur_ptrs.data();
        auto planner = make_unique<MRMHAPlanner>( space.get(),
                heurs[0].get(), inad, n_heurs-1);
        //auto planner = make_unique<MHAPlanner>( space.get(),
                //heurs[0].get(), inad, n_heurs-1 );
        //auto planner = make_unique<ARAStar>( space.get(), heurs[0].get() );
        //planner->force_planning_from_scratch();
        planner->set_initial_mha_eps(2);
        planner->set_initialsolution_eps(3);
        //planner->setTargetEpsilon(3);
        planner->set_search_mode(false);

        smpl::GoalConstraint goal;
        goal.type = smpl::GoalType::XYZ_RPY_GOAL;
        goal.pose = goal_pose;
        goal.xyz_tolerance[0] = 0.03;
        goal.xyz_tolerance[1] = 0.03;
        goal.xyz_tolerance[2] = 0.03;
        goal.rpy_tolerance[0] = 3.14;
        goal.rpy_tolerance[1] = 3.14;
        goal.rpy_tolerance[2] = 3.14;

        std::vector<smpl::RobotHeuristic*> hs;
        for(auto& h: heurs)
            hs.push_back(h.get());

        ROS_INFO("Setting Goal");
        if(!setGoal( goal, space.get(), hs, planner.get() ))
            return 0;
        ROS_INFO("Setting Start");
        if(!setStart( start_state, rm.get(), space.get(), hs, planner.get() ))
            return 0;

        //SV_SHOW_INFO(dynamic_cast<BfsFullbodyHeuristic*>(hs[0])->get2DValuesVisualization());
        // plan
        ROS_INFO("Calling planner");
        std::vector<int> soltn_ids;
        int soltn_cost;

        planner->force_planning_from_scratch();

        // plan
        double allowed_planning_time;
        ph.param("allowed_planning_time", allowed_planning_time, 30.0);

        auto success = planner->replan(allowed_planning_time, &soltn_ids, &soltn_cost);
        if(!success){
            ROS_ERROR("Failed to plan.");
        }else{
            ROS_INFO("Planning successful");
            ROS_INFO("Solution Cost: %d \n", soltn_cost);
            ROS_INFO("Animate path");
            std::vector<RobotState> soltn_path;
            space->extractPath( soltn_ids, soltn_path );

            visualization_msgs::MarkerArray whole_path;
            std::vector<visualization_msgs::Marker> m_all;

            int idx = 0;
            for( int pidx=0; pidx<soltn_ids.size(); pidx++ ){
                auto& state = soltn_path[pidx];
                auto markers = cc.getCollisionRobotVisualization(state);
                for (auto& m : markers.markers) {
                    m.ns = "path_animation";
                    m.id = idx;
                    idx++;
                    whole_path.markers.push_back(m);
                }
                visualizer.visualize(smpl::visual::Level::Info, markers);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            ROS_INFO("Solution Path:");
            for( auto& state: soltn_path ){
                printVector<double>(state);
            }
        }

    }
}

int MsgSubscriber::plan_mrmha(
        ros::NodeHandle nh,
        ros::NodeHandle ph,
        geometry_msgs::Pose grasp ){
    int req, done;
    ros::param::get("/walker_planner_request", req);
    if (req) {
        ros::param::set("/walker_planner_request", 0);

        ROS_INFO("Initialize visualizer");
        smpl::VisualizerROS visualizer(nh, 100);
        smpl::viz::set_visualizer(&visualizer);

        // Let publishers set up
        ros::Duration(1.0).sleep();

        // Read in start state from file and update the scene...
        // Start state is also required by the planner...
        moveit_msgs::RobotState start_state;
        if (!ReadInitialConfiguration(ph, start_state)) {
            ROS_ERROR("Failed to get initial configuration.");
            return 1;
        }

        bool read_goal_from_file;
        ph.param("read_goal_from_file", read_goal_from_file, true );
        if( !read_goal_from_file ){
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
        }

        smpl::Affine3 goal_pose;
         // Read from file.
         if (read_goal_from_file) {
             std::vector<double> goal_state( 6, 0 );
             ph.param("goal/x", goal_state[0], 0.0);
             ph.param("goal/y", goal_state[1], 0.0);
             ph.param("goal/z", goal_state[2], 0.0);
             ph.param("goal/roll", goal_state[3], 0.0);
             ph.param("goal/pitch", goal_state[4], 0.0);
             ph.param("goal/yaw", goal_state[5], 0.0);

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
         } else{
             tf::poseMsgToEigen( grasp, goal_pose );
         }

        smpl::GoalConstraint goal;
        goal.type = smpl::GoalType::XYZ_RPY_GOAL;
        goal.pose = goal_pose;
        goal.xyz_tolerance[0] = 0.03;
        goal.xyz_tolerance[1] = 0.03;
        goal.xyz_tolerance[2] = 0.03;
        goal.rpy_tolerance[0] = 3.14;
        goal.rpy_tolerance[1] = 3.14;
        goal.rpy_tolerance[2] = 3.14;

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

        std::string planning_mode = "FULLBODY";
        ros::param::get("/walker_planner_mode", planning_mode);

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
        auto df_origin_x = 0;//m_start_base.position.x - 5;
        auto df_origin_y = 0;//m_start_base.position.y - 4;
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

        grid.addPointsToField(m_occgrid_points);
        ROS_ERROR("Occupied voxesls: %d", grid.getOccupiedVoxelCount());

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

        /////////////////
        // Setup Scene //
        /////////////////

        ROS_INFO("Initialize scene");

        scene.SetCollisionSpace(&cc);

        auto objects = GetMultiRoomMapCollisionCubes(planning_frame, 20, 15, .8, 1);
        for (auto& object : objects) {
            scene.ProcessCollisionObjectMsg(object);
        }

        ROS_INFO("Setting up robot model");
        auto rm = SetupRobotModel(robot_description, robot_config);
        if (!rm) {
            ROS_ERROR("Failed to set up Robot Model");
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

        if( planning_mode == "FULLBODY" ){
            bool ret = scene.ProcessOctomapMsg(m_map_with_pose);
            if(ret)
                ROS_INFO("Succesfully added octomap");
            else
                ROS_INFO("Could not added octomap");
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
        if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)){
            ROS_ERROR("Failed to read planner config");
            return 1;
        }
        planning_config.cost_per_cell = 1000;

        auto resolutions = getResolutions( rm.get(), planning_config );
        auto actions_full = make_unique<ManipLatticeActionSpace>();
        auto actions_base = make_unique<ManipLatticeActionSpace>();
        auto actions_arm = make_unique<ManipLatticeActionSpace>();
        auto space = make_unique<smpl::ManipLatticeMultiRep>();

        //XXX Assuming:
        //1. Anchor get full
        //2. Inad1 gets arm
        //3. Inad2 gets base
        std::vector<ActionSpace*> v_actions;
        v_actions.push_back(actions_full.get());
        //v_actions.push_back(actions_full.get());
        v_actions.push_back(actions_arm.get());
        v_actions.push_back(actions_base.get());
        if (!space->init( rm.get(), &cc, resolutions, v_actions )) {
            SMPL_ERROR("Failed to initialize Manip Lattice");
            return 1;
        }

        for(auto& actions : v_actions){
            if (!actions->init(space.get())) {
                SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
                return 1;
            }
        }

        space->setVisualizationFrameId(grid.getReferenceFrame());

        actions_full->useMultipleIkSolutions(planning_config.use_multiple_ik_solutions);
        actions_full->useAmp(MotionPrimitive::SNAP_TO_XYZ, planning_config.use_xyz_snap_mprim);
        actions_full->useAmp(MotionPrimitive::SNAP_TO_RPY, planning_config.use_rpy_snap_mprim);
        actions_full->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.use_xyzrpy_snap_mprim);
        actions_full->useAmp(MotionPrimitive::SHORT_DISTANCE, planning_config.use_short_dist_mprims);
        actions_full->ampThresh(MotionPrimitive::SNAP_TO_XYZ, planning_config.xyz_snap_dist_thresh);
        actions_full->ampThresh(MotionPrimitive::SNAP_TO_RPY, planning_config.rpy_snap_dist_thresh);
        actions_full->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.xyzrpy_snap_dist_thresh);
        actions_full->ampThresh(MotionPrimitive::SHORT_DISTANCE, planning_config.short_dist_mprims_thresh);
        actions_base->useMultipleIkSolutions(planning_config.use_multiple_ik_solutions);
        actions_base->useAmp(MotionPrimitive::SNAP_TO_XYZ, planning_config.use_xyz_snap_mprim);
        actions_base->useAmp(MotionPrimitive::SNAP_TO_RPY, planning_config.use_rpy_snap_mprim);
        actions_base->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.use_xyzrpy_snap_mprim);
        actions_base->useAmp(MotionPrimitive::SHORT_DISTANCE, planning_config.use_short_dist_mprims);
        actions_base->ampThresh(MotionPrimitive::SNAP_TO_XYZ, planning_config.xyz_snap_dist_thresh);
        actions_base->ampThresh(MotionPrimitive::SNAP_TO_RPY, planning_config.rpy_snap_dist_thresh);
        actions_base->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.xyzrpy_snap_dist_thresh);
        actions_base->ampThresh(MotionPrimitive::SHORT_DISTANCE, planning_config.short_dist_mprims_thresh);
        actions_arm->useMultipleIkSolutions(planning_config.use_multiple_ik_solutions);
        actions_arm->useAmp(MotionPrimitive::SNAP_TO_XYZ, planning_config.use_xyz_snap_mprim);
        actions_arm->useAmp(MotionPrimitive::SNAP_TO_RPY, planning_config.use_rpy_snap_mprim);
        actions_arm->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.use_xyzrpy_snap_mprim);
        actions_arm->useAmp(MotionPrimitive::SHORT_DISTANCE, planning_config.use_short_dist_mprims);
        actions_arm->ampThresh(MotionPrimitive::SNAP_TO_XYZ, planning_config.xyz_snap_dist_thresh);
        actions_arm->ampThresh(MotionPrimitive::SNAP_TO_RPY, planning_config.rpy_snap_dist_thresh);
        actions_arm->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.xyzrpy_snap_dist_thresh);
        actions_arm->ampThresh(MotionPrimitive::SHORT_DISTANCE, planning_config.short_dist_mprims_thresh);

        // XXX Loads from filename
        std::vector<std::string> mprim_filenames;
        std::stringstream ss(planning_config.mprim_filenames);
        std::string temp;
        while(getline(ss, temp, ',')){
            mprim_filenames.push_back(temp);
            ROS_ERROR("mprim_filename: %s", temp.c_str());
        }

        if (!actions_full->load(mprim_filenames[0])) {
            return 1;
        }
        if (!actions_base->load(mprim_filenames[1])) {
            return 1;
        }
        if (!actions_arm->load(mprim_filenames[2])) {
            return 1;
        }

        //////////////
        // Planning //
        //////////////


        std::vector<std::unique_ptr<smpl::RobotHeuristic>> heurs;
        if(!constructHeuristics( heurs, space, grid, rm, goal_pose, planning_config )){
            ROS_ERROR("Could not construct heuristics.");
            return 0;
        }

        ROS_INFO("Constructing Planner");
        std::vector<Heuristic*> heur_ptrs;
        int n_heurs = heurs.size();
        for(int i=1; i<n_heurs; i++)
            heur_ptrs.push_back(heurs[i].get());
        ROS_INFO("Number of inadmissible heuristics: %d", n_heurs-1);

        Heuristic** inad = heur_ptrs.data();
        planner->set_initial_mha_eps(2);
        planner->set_initialsolution_eps(3);
        //planner->setTargetEpsilon(3);
        planner->set_search_mode(false);

        std::vector<smpl::RobotHeuristic*> hs;
        for(auto& h: heurs)
            hs.push_back(h.get());

        ROS_INFO("Setting Goal");
        if(!setGoal( goal, space.get(), hs, planner.get() ))
            return 0;
        ROS_INFO("Setting Start");
        if(!setStart( start_state, rm.get(), space.get(), hs, planner.get() ))
            return 0;

        //SV_SHOW_INFO(dynamic_cast<BfsFullbodyHeuristic*>(hs[0])->get2DValuesVisualization());
        // plan
        ROS_INFO("Calling planner");
        std::vector<int> soltn_ids;
        int soltn_cost;

        planner->force_planning_from_scratch();

        // plan
        double allowed_planning_time;
        ph.param("allowed_planning_time", allowed_planning_time, 30.0);

        auto success = planner->replan(allowed_planning_time, &soltn_ids, &soltn_cost);
        if(!success){
            ROS_ERROR("Failed to plan.");
        }else{
            ROS_INFO("Planning successful");
            ROS_INFO("Solution Cost: %d \n", soltn_cost);
            ROS_INFO("Animate path");
            std::vector<RobotState> soltn_path;
            space->extractPath( soltn_ids, soltn_path );

            visualization_msgs::MarkerArray whole_path;
            std::vector<visualization_msgs::Marker> m_all;

            int idx = 0;
            for( int pidx=0; pidx<soltn_ids.size(); pidx++ ){
                auto& state = soltn_path[pidx];
                auto markers = cc.getCollisionRobotVisualization(state);
                for (auto& m : markers.markers) {
                    m.ns = "path_animation";
                    m.id = idx;
                    idx++;
                    whole_path.markers.push_back(m);
                }
                visualizer.visualize(smpl::visual::Level::Info, markers);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            ROS_INFO("Solution Path:");
            for( auto& state: soltn_path ){
                printVector<double>(state);
            }
        }
    }
}
