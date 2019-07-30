#include "walker_planner.h"
#include <smpl/ros/planner_interface.h>

using namespace smpl;

MsgSubscriber::MsgSubscriber(ros::NodeHandle nh, ros::NodeHandle ph) {
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
    //m_sub_octomap = m_nh.subscribe("/octomap_binary", 1000, &MsgSubscriber::octomapCallback, this);
    m_sub_pose = m_nh.subscribe("/Grasps", 1000, &MsgSubscriber::poseCallback, this);
}

void MsgSubscriber::octomapCallback(const octomap_msgs::Octomap& msg) {
    ROS_ERROR("Octomap received");
    m_map_with_pose.header.frame_id = "world";
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

void MsgSubscriber::occgridCallback(const nav_msgs::OccupancyGrid& msg) {
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

void MsgSubscriber::poseCallback(const geometry_msgs::PoseStamped grasp) {
    m_grasp = grasp.pose;
    m_grasp_received = true;

    std::string planning_mode = "FULLBODY";
    ros::param::get("/walker_planner_mode", planning_mode);
    if( ( planning_mode == "FULLBODY" ) && m_start_received){// && m_occgrid_received){// &&  m_octomap_received) {
        ROS_ERROR("Fullbody Planner Called.");
        m_octomap_received = false;
        m_start_received = false;
        m_grasp_received = false;
        plan(m_nh, m_ph, m_grasp);
    }
    else if( ( planning_mode == "BASE" ) && m_start_received  &&
            m_occgrid_received ) {
        ROS_ERROR("Base Planner Called.");
        plan(m_nh, m_ph, m_grasp);
    }
    else{
        ROS_ERROR("Planner mode not understood.");
    }
}

void MsgSubscriber::startCallback(const geometry_msgs::PoseWithCovarianceStamped start) {
    if( m_start_received == false )
        ROS_INFO("Start received");
    m_start_base = start.pose.pose;
    m_start_received = true;
}

octomap_msgs::OctomapWithPose MsgSubscriber::subscribeOctomap(ros::NodeHandle nh) {
    return m_map_with_pose;
}

geometry_msgs::Pose MsgSubscriber::subscribeGrasp(ros::NodeHandle nh) {
    return m_grasp;
}


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

bool setGoal(const smpl::GoalConstraint& goal,
        ManipLattice* pspace,
        std::vector<RobotHeuristic*>& heurs,
        SBPLPlanner* planner ){

        double yaw, pitch, roll;
        angles::get_euler_zyx(goal.pose.rotation(), yaw, pitch, roll);

    // set sbpl environment goal
    ROS_INFO("Setting goal in pspace");
    if (!pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return false;
    }

    ROS_INFO("Updating goal in heuristics.");
    for (auto& h : heurs) {
        h->updateGoal(goal);
    }
    auto h = heurs[0]->GetGoalHeuristic(0);
    ROS_ERROR("Computed heur: :%d", h);

    // set planner goal
    auto goal_id = pspace->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    ROS_INFO("Setting planner Goal");
    if (planner->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return false;
    }

    return true;
}

// Convert the input robot state to an SMPL robot state and update the start
// state in the graph, heuristic, and search.
bool setStart(const moveit_msgs::RobotState& state,
        smpl::RobotModel* robot,
        ManipLattice* pspace,
        std::vector<RobotHeuristic*>& heurs,
        SBPLPlanner* planner ){

    if (!state.multi_dof_joint_state.joint_names.empty()) {
        auto& mdof_joint_names = state.multi_dof_joint_state.joint_names;
        for (auto& joint_name : robot->getPlanningJoints()) {
            auto it = std::find(begin(mdof_joint_names), end(mdof_joint_names), joint_name);
            if (it != end(mdof_joint_names)) {
                ROS_WARN("planner does not currently support planning for multi-dof joints. found '%s' in planning joints", joint_name.c_str());
            }
        }
    }

    RobotState initial_positions;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            state.joint_state,
            state.multi_dof_joint_state,
            robot->getPlanningJoints(),
            initial_positions,
            missing))
    {
        ROS_WARN_STREAM("start state is missing planning joints: " << missing);

        moveit_msgs::RobotState fixed_state = state;
        for (auto& variable : missing) {
            ROS_WARN("  Assume position 0.0 for joint variable '%s'", variable.c_str());
            fixed_state.joint_state.name.push_back(variable);
            fixed_state.joint_state.position.push_back(0.0);
        }

        if (!leatherman::getJointPositions(
                fixed_state.joint_state,
                fixed_state.multi_dof_joint_state,
                robot->getPlanningJoints(),
                initial_positions,
                missing))
        {
            return false;
        }

//        return false;
    }

    if (!pspace->setStart(initial_positions)) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    auto start_id = pspace->getStartStateID();
    if (start_id == -1) {
        ROS_ERROR("No start state has been set");
        return false;
    }

    for (auto& h : heurs) {
        h->updateStart(initial_positions);
    }

    if (planner->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    return true;
}

int MsgSubscriber::plan(
        ros::NodeHandle nh,
        ros::NodeHandle ph,
        geometry_msgs::Pose grasp ){
    if( m_planner_mode == PLANNER_MODE::MHA )
        plan_mha( nh, ph, grasp );
    else if( m_planner_mode == PLANNER_MODE::MRMHA )
        plan_mrmha( nh, ph, grasp );
    else
        throw "Planner mode not understood.";
}

int MsgSubscriber::plan_mha(
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
        if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)) {
            ROS_ERROR("Failed to read planner config");
            return 1;
        }

        smpl::PlannerInterface planner(rm.get(), &cc, &grid);

        smpl::PlanningParams params;

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
        params.addParam("bfs_cost_per_cell", 200);
        params.addParam("x_coeff", 1.0);
        params.addParam("y_coeff", 1.0);
        params.addParam("z_coeff", 1.0);
        params.addParam("rot_coeff", 1.0);
        //params.addParam("interpolate_path", true);
        params.addParam("shortcut_path", true);

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
            FillGoalConstraint(goal_state, planning_frame, req.goal_constraints[0], 0.2, 0.2);
        else
            FillGoalConstraint(goal_state, planning_frame, req.goal_constraints[0], 0.10, 2*3.14);

        req.group_name = robot_config.group_name;
        req.max_acceleration_scaling_factor = 1.0;
        req.max_velocity_scaling_factor = 1.0;
        req.num_planning_attempts = 1;
    //    req.path_constraints;
        if (planning_mode == "BASE")
            req.planner_id = "arastar.euclid_diff.manip";
        else
            req.planner_id = "mhastar.euclid.bfs.manip";
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

void MsgSubscriber::publish_path(
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

int main(int argc, char* argv[]){
    ros::init(argc, argv, "walker_planner");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Rate loop_rate(10);

    MsgSubscriber sub(nh, ph);
    sub.m_planner_mode = PLANNER_MODE::MRMHA;
    while(ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
