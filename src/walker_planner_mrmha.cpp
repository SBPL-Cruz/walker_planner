 #include <tf2/LinearMath/Quaternion.h>

#include <smpl/search/awastar.h>
#include <sbpl/planners/mrmhaplanner.h>
#include <sbpl/planners/araplanner.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/arm_retract_heuristic.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/manip_lattice_multi_rep.h>

#include "walker_planner.h"

using namespace smpl;

bool constructHeuristics(
        std::vector<std::unique_ptr<RobotHeuristic>>& heurs,
        std::unique_ptr<ManipLatticeMultiRep>& pspace,
        smpl::OccupancyGrid& grid,
        PlannerConfig& params ){

    SMPL_INFO("Initialize Heuristics");

    heurs.clear();
    {
        auto h = make_unique<BfsHeuristic>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        //heurs.push_back(std::move(h));
    }

    {
        auto h = make_unique<EuclidDistHeuristic>();
        if (!h->init(pspace.get())) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        h->setWeightRot(0.1);
        heurs.push_back(std::move(h));
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

    for (auto& entry : heurs) {
        pspace->insertHeuristic(entry.get());
    }
    return true;
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

        /*
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
        */
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
        /////////////////
        // Setup Scene //
        /////////////////

        ROS_INFO("Initialize scene");

        scene.SetCollisionSpace(&cc);

        std::string object_filename;
        ph.getParam("object_filename", object_filename);
        ROS_ERROR("%s", object_filename.c_str());

        // Read in collision objects from file and add to the scene...
        //if (!object_filename.empty()) {
        //    auto objects = GetCollisionObjects(object_filename, planning_frame);
        //    for (auto& object : objects) {
        //        scene.ProcessCollisionObjectMsg(object);
        //    }
        //}
        /*
        auto objects = GetMultiRoomMapCollisionCubes(planning_frame, 20, 15, .8);
        for (auto& object : objects) {
            scene.ProcessCollisionObjectMsg(object);
        }
        */

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
        if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)){
            ROS_ERROR("Failed to read planner config");
            return 1;
        }
        planning_config.cost_per_cell = 1000;

        auto resolutions = getResolutions( rm.get(), planning_config );
        auto actions = make_unique<ManipLatticeActionSpace>();
        auto space = make_unique<smpl::ManipLatticeMultiRep>();

        std::vector<ActionSpace*> v_actions;
        v_actions.push_back(actions.get());
        v_actions.push_back(actions.get());
        v_actions.push_back(actions.get());
        if (!space->init( rm.get(), &cc, resolutions, v_actions )) {
            SMPL_ERROR("Failed to initialize Manip Lattice");
            return 1;
        }

        if (!actions->init(space.get())) {
            SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
            return 1;
        }

        space->setVisualizationFrameId(grid.getReferenceFrame());

        actions->useMultipleIkSolutions(planning_config.use_multiple_ik_solutions);
        actions->useAmp(MotionPrimitive::SNAP_TO_XYZ, planning_config.use_xyz_snap_mprim);
        actions->useAmp(MotionPrimitive::SNAP_TO_RPY, planning_config.use_rpy_snap_mprim);
        actions->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.use_xyzrpy_snap_mprim);
        actions->useAmp(MotionPrimitive::SHORT_DISTANCE, planning_config.use_short_dist_mprims);
        actions->ampThresh(MotionPrimitive::SNAP_TO_XYZ, planning_config.xyz_snap_dist_thresh);
        actions->ampThresh(MotionPrimitive::SNAP_TO_RPY, planning_config.rpy_snap_dist_thresh);
        actions->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.xyzrpy_snap_dist_thresh);
        actions->ampThresh(MotionPrimitive::SHORT_DISTANCE, planning_config.short_dist_mprims_thresh);
        // XXX Loads from filename
        if (!actions->load(planning_config.mprim_filename)) {
            return 1;
        }

        //////////////
        // Planning //
        //////////////


        std::vector<std::unique_ptr<smpl::RobotHeuristic>> heurs;
        if(!constructHeuristics( heurs, space, grid, planning_config )){
            ROS_ERROR("Could not construct heuristics.");
            return 0;
        }

        ROS_INFO("Constructing Planner");
        std::vector<Heuristic*> heur_ptrs;
        int n_heurs = heurs.size();
        for(int i=1; i<n_heurs; i++)
            heur_ptrs.push_back(heurs[i].get());

        Heuristic** inad = heur_ptrs.data();
        //auto planner = make_unique<MRMHAPlanner>( space.get(),
        //        heurs[0].get(), inad, n_heurs-1);
        //auto planner = make_unique<MHAPlanner>( space.get(),
        //        heurs[0].get(), inad, n_heurs-1 );
        auto planner = make_unique<AWAStar>( space.get(), heurs[0].get() );
        planner->set_initialsolution_eps(10);
        //planner->set_initial_mha_eps(50);
        planner->set_search_mode(false);

        smpl::GoalConstraint goal;
        goal.type = smpl::GoalType::XYZ_RPY_GOAL;
        goal.pose = goal_pose;
        goal.xyz_tolerance[0] = 0.2;
        goal.xyz_tolerance[1] = 0.2;
        goal.xyz_tolerance[2] = 0.2;
        goal.rpy_tolerance[0] = 0.7;
        goal.rpy_tolerance[1] = 0.7;
        goal.rpy_tolerance[2] = 0.7;

        std::vector<smpl::RobotHeuristic*> hs;
        for(auto& h: heurs)
            hs.push_back(h.get());

        ROS_INFO("Setting Goal");
        if(!setGoal( goal, space.get(), hs, planner.get() ))
            return 0;
        ROS_INFO("Setting Start");
        if(!setStart( start_state, rm.get(), space.get(), hs, planner.get() ))
            return 0;

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
        }
    /*
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
    */
    }
}
