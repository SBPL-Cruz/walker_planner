#include <sbpl/planners/mrmhaplanner.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>

#include "walker_planner.h"

using namespace smpl;

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
        SMPL_ERROR("Parameter 'discretization' not found in planning params");
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
                SMPL_ERROR("Discretization for variable '%s' not found in planning parameters", vname.c_str());
            }
            resolutions[vidx] = dit->second;
        } else {
            auto dit = disc.find(vname);
            if (dit == end(disc)) {
                SMPL_ERROR("Discretization for variable '%s' not found in planning parameters", vname.c_str());
            }
            resolutions[vidx] = dit->second;
        }

        SMPL_DEBUG("resolution(%s) = %0.3f", vname.c_str(), resolutions[vidx]);
    }

    return resolutions;
}

std::unique_ptr<MRMHAPlanner> constructPlanner(
        std::vector<std::unique_ptr<RobotHeuristic>>& heurs,
        std::unique_ptr<ManipLattice>& pspace,
        smpl::OccupancyGrid& grid,
        PlannerConfig& params ){

    SMPL_INFO("Initialize Heuristics");

    heurs.clear();
    {
        auto h = make_unique<BfsHeuristic>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            return false;
        }
        heurs.push_back(std::move(h));
    }

    // Need at least 2 heuristics for MR-MHA to make sense.
    {
        auto h = make_unique<BfsHeuristic>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            return false;
        }
        heurs.push_back(std::move(h));
    }

    for (auto& entry : heurs) {
        pspace->insertHeuristic(entry.get());
    }

    std::vector<Heuristic*> heur_ptrs;
    for(auto& entry: heurs)
        heur_ptrs.push_back(entry.get());

    SMPL_INFO("Create planner");
    auto planner = make_unique<MRMHAPlanner>( pspace.get(),
            heur_ptrs[0], &(heur_ptrs[1]), heur_ptrs.size() - 1 );
    return planner;
}

int MsgSubscriber::plan_mrmha(
        ros::NodeHandle nh,
        ros::NodeHandle ph,
        geometry_msgs::Pose grasp,
        octomap_msgs::OctomapWithPose octomap ){
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
            ros::param::set("/test_walker_interface/robot_model/chain_tip_link", "base_link");
            ros::param::set("/test_walker_interface/robot_model/planning_joints", "x y theta");
            std::string pkg_path = ros::package::getPath("walker_planner");
            ros::param::set("/test_walker_interface/planning/mprim_filename", pkg_path + "/config/walker_base.mprim");
        }
        else {
          ROS_INFO("Switching to FULLBODY planning mode.");
          ros::param::set("/test_walker_interface/robot_model/chain_tip_link", "right_palm_link");
          ros::param::set("/test_walker_interface/robot_model/planning_joints", "x  y theta right_limb_j1  right_limb_j2 right_limb_j3 right_limb_j4 right_limb_j5 right_limb_j6 right_limb_j7" );
          std::string pkg_path = ros::package::getPath("walker_planner");
          ros::param::set("/test_walker_interface/planning/mprim_filename", pkg_path + "/config/walker.mprim");
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
        ph.getParam("object_filename", object_filename);
        ROS_ERROR("%s", object_filename.c_str());

        // Read in collision objects from file and add to the scene...
        //if (!object_filename.empty()) {
        //    auto objects = GetCollisionObjects(object_filename, planning_frame);
        //    for (auto& object : objects) {
        //        scene.ProcessCollisionObjectMsg(object);
        //    }
        //}
        auto objects = GetMultiRoomMapCollisionCubes(planning_frame, 20, 15, .8);
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

        auto resolutions = getResolutions( rm.get(), planning_config );
        auto actions = make_unique<ManipLatticeActionSpace>();
        auto space = make_unique<ManipLattice>();

        if (!space->init( rm.get(), &cc, resolutions, actions.get() )) {
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

        if (!actions->load(planning_config.mprim_filename)) {
            return 1;
        }

        /*
        SMPL_DEBUG_NAMED(PI_LOGGER, "Action Set:");
        for (auto ait = actions.begin(); ait != actions.end(); ++ait) {
            SMPL_DEBUG_NAMED(PI_LOGGER, "  type: %s", to_cstring(ait->type));
            if (ait->type == MotionPrimitive::SNAP_TO_RPY) {
                SMPL_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
                SMPL_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_RPY));
            } else if (ait->type == MotionPrimitive::SNAP_TO_XYZ) {
                SMPL_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
                SMPL_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ));
            } else if (ait->type == MotionPrimitive::SNAP_TO_XYZ_RPY) {
                SMPL_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
                SMPL_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY));
            } else if (ait->type == MotionPrimitive::LONG_DISTANCE ||
                ait->type == MotionPrimitive::SHORT_DISTANCE)
            {
                SMPL_DEBUG_STREAM_NAMED(PI_LOGGER, "    action: " << ait->action);
            }
        }
        */

        //////////////
        // Planning //
        //////////////

        moveit_msgs::MotionPlanRequest req;
        moveit_msgs::MotionPlanResponse res;

        ph.param("allowed_planning_time", req.allowed_planning_time, 30.0);
        req.goal_constraints.resize(1);
        if(planning_mode == "BASE")
            FillGoalConstraint(goal_state, planning_frame, req.goal_constraints[0], 0.2, 0.2);
        else
            FillGoalConstraint(goal_state, planning_frame, req.goal_constraints[0], 0.40, 2*3.14);

        req.group_name = robot_config.group_name;
        req.max_acceleration_scaling_factor = 1.0;
        req.max_velocity_scaling_factor = 1.0;
        req.num_planning_attempts = 1;


        std::vector<std::unique_ptr<RobotHeuristic>> heurs;
        auto planner = constructPlanner( heurs, space, grid, planning_config );

        planner->set_initial_mha_eps(10);
        planner->set_initialsolution_eps(10);
        planner->set_search_mode(false);

        //if (planning_mode == "BASE")
        //    throw "Not Implemented";
        //else
        //    req.planner_id = "mrmhastar.euclid.euclid.arm_retract.manip_mr";
        req.start_state = start_state;

        // plan
        //ROS_INFO("Calling solve...");
        //moveit_msgs::PlanningScene planning_scene;
        //planning_scene.robot_state = start_state;
        //if (!planner.solve(planning_scene, req, res)) {
        //    ROS_ERROR("Failed to plan.");

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
            //return 1;
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
