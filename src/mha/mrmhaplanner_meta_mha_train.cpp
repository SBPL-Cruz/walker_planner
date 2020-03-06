#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <memory>
#include <fstream>
#include <stdlib.h>
#include <time.h>
#include <algorithm>

#include <ros/ros.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/manip_lattice_multi_rep.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/euclid_diffdrive_heuristic.h>
#include <smpl/heuristic/arm_retract_heuristic.h>
#include <smpl/heuristic/base_rot_bfs_heuristic.h>
//#include <sbpl/planners/mrmhaplanner.h>
#include <smpl/search/smhastar.h>
#include <sbpl/planners/types.h>

#include "heuristics/walker_heuristics.h"
#include "planners/mrmhaplanner_meta_mha.h"
#include "motion_planner.h"
#include "motion_planner_ros.h"
#include "scheduling_policies.h"
#include "utils/utils.h"

class ReadExperimentsFromFile {
    public:
    ReadExperimentsFromFile(ros::NodeHandle);
    moveit_msgs::RobotState getStart(PlanningEpisode);
    smpl::GoalConstraint getGoal(PlanningEpisode);
    bool canCallPlanner() const { return true; }
    int numEpisodes()
    {
        return std::min(m_start_states.size(), m_goal_constraints.size());
    }

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

class StartGoalDatabase
{
    public:
    StartGoalDatabase(ros::NodeHandle);
    void setReferenceState(moveit_msgs::RobotState ref_state);
    void addStartGoal(std::vector<double> start, smpl::Affine3 goal);
    void addStartGoal(std::vector<double> start, smpl::GoalConstraint goal);
    moveit_msgs::RobotState getStart(PlanningEpisode);
    smpl::GoalConstraint getGoal(PlanningEpisode);
    bool canCallPlanner() const { return true; }
    int numEpisodes()
    {
        return m_start_states.size();
    }

    public:
    moveit_msgs::RobotState vectorToRobotState(std::vector<double> _state);
    smpl::GoalConstraint affineToGoalConstraint(smpl::Affine3 _goal);

    ros::NodeHandle m_nh;
    moveit_msgs::RobotState m_ref_state;
    std::vector<moveit_msgs::RobotState> m_start_states;
    std::vector<smpl::GoalConstraint> m_goal_constraints;
};

StartGoalDatabase::StartGoalDatabase(ros::NodeHandle _nh) :
    m_nh{_nh}
{ }

void StartGoalDatabase::setReferenceState(moveit_msgs::RobotState _ref_state)
{
    m_ref_state = _ref_state;
}

void StartGoalDatabase::addStartGoal(std::vector<double> _start, smpl::Affine3 _goal)
{
    auto robot_state = vectorToRobotState(_start);
    m_start_states.push_back(robot_state);

    auto goal_const = affineToGoalConstraint(_goal);
    m_goal_constraints.push_back(goal_const);
}

void StartGoalDatabase::addStartGoal(std::vector<double> _start,
        smpl::GoalConstraint _goal)
{
    auto robot_state = vectorToRobotState(_start);
    m_start_states.push_back(robot_state);

    m_goal_constraints.push_back(_goal);
}

moveit_msgs::RobotState StartGoalDatabase::vectorToRobotState(std::vector<double> _state)
{
    moveit_msgs::RobotState robot_state;

    for(int i = 0; i < _state.size(); i++)
    {
        robot_state.joint_state.name.push_back(m_ref_state.joint_state.name[i]);
        robot_state.joint_state.position.push_back(_state[i]);
    }

    return robot_state;
}

smpl::GoalConstraint StartGoalDatabase::affineToGoalConstraint(smpl::Affine3 _goal)
{
    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::XYZ_RPY_GOAL;
    goal.pose = _goal;
    goal.xyz_tolerance[0] = 0.05;
    goal.xyz_tolerance[1] = 0.05;
    goal.xyz_tolerance[2] = 0.05;
    goal.rpy_tolerance[0] = 0.39;
    goal.rpy_tolerance[1] = 0.39;
    goal.rpy_tolerance[2] = 0.39;

    return goal;
}

moveit_msgs::RobotState StartGoalDatabase::getStart(PlanningEpisode _ep)
{
    return m_start_states[_ep];
}

smpl::GoalConstraint StartGoalDatabase::getGoal(PlanningEpisode _ep)
{
    return m_goal_constraints[_ep];
}

void writePath(std::string _file_name, std::string _header, std::vector<smpl::RobotState> _path){
    std::ofstream file;
    file.open(_file_name, std::ios::out);
    if(!file)
        ROS_ERROR("Could not open file.");
    file<<_header<<"\n";

    for(const auto& state : _path){
        for(const auto& val : state){
            file<<val<<" ";
        }
        file<<"\n";
    }

    file.close();
}

void writeStartGoals(std::string _start_file_name, std::string _goal_file_name,
        std::string _header, std::vector<smpl::RobotState> _starts,
        std::vector<smpl::Affine3> _goals)
{
    std::ofstream file;
    file.open(_start_file_name, std::ios::app);
    if(!file)
        ROS_ERROR("Could not open file");

    file<< _header<< "\n";

    for(const auto& state : _starts)
    {
        for(const auto& val : state)
            file<< val<< " ";
        file<< "\n";
    }
    file.close();

    file.open(_goal_file_name, std::ios::app);
    if(!file)
        ROS_ERROR("Could not open goal file");
    for(const auto& pose : _goals)
    {
        auto xyzrpy = poseToXYZRPY(pose);
        for(int i = 0; i < 6; i++)
            file<< xyzrpy[i]<< " ";
        file<< "\n";
    }
    file.close();
}

int main(int argc, char** argv) {
    SMPL_INFO("Generating Training data for Meta-MHA Policy");
    std::string instance(argv[1]);
    std::string node_name = "mrmhaplanner_meta_mha_train";// + instance;
    SMPL_INFO("Running node %s", node_name.c_str());
    ros::init(argc, argv, node_name, ros::init_options::AnonymousName);
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

    auto df_size_x = 6.0;//20.0;
    auto df_size_y = 6.0;//15.0;
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
    cc.setPadding(0.03);
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
    ROS_INFO("Initialize scene");

    scene_ptr->SetCollisionSpace(&cc);

    ROS_INFO("Setting up robot model");
    auto rm = SetupRobotModel(robot_description, robot_config);
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
    auto action_space = std::make_unique<smpl::ManipLatticeMultiActionSpace>(NUM_ACTION_SPACES);
    auto space = std::make_unique<smpl::ManipLatticeMultiRep>();

    //auto action_space = std::make_unique<smpl::ManipLatticeActionSpace>();
    //auto space = std::make_unique<smpl::ManipLattice>();

    if (!space->init( rm.get(), &cc, resolutions, action_space.get() )) {
        ROS_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }

    if (!action_space->init(space.get())) {
        ROS_ERROR("Failed to initialize Manip Lattice Multi Action Space");
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
    for( int i=0; i<NUM_ACTION_SPACES; i++ )
        if(!action_space->load(i, mprim_filenames[i]))
            return 1;

    //if(!action_space->load(mprim_filenames[0])){
        //SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
        //return 1;
    //}

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
    ROS_ERROR("Use Snap: %d", planning_config.use_xyz_snap_mprim);

    // Save start-goal in StartGoalDatabase

    ///////////////
    //Planning////
    /////////////

    std::array< std::shared_ptr<smpl::RobotHeuristic>, NUM_QUEUES > robot_heurs;
    std::vector< std::shared_ptr<smpl::RobotHeuristic> > bfs_heurs;

    std::array<int, NUM_QUEUES> rep_ids;

    if(!constructHeuristicsFullbody( robot_heurs, rep_ids, bfs_heurs, space.get(), grid_ptr.get(), rm.get(), planning_config )){
        ROS_ERROR("Could not construct heuristics.");
        return 0;
    }

    ROS_ERROR("Number of heuristics: %d", robot_heurs.size());
    assert(robot_heurs.size() == NUM_QUEUES);

    assert(robot_heurs[0] != nullptr);

    std::vector<smpl::RobotHeuristic*> inad_heurs;
    for(int i = 1; i < robot_heurs.size(); i++)
        inad_heurs.push_back(robot_heurs[i].get());

    std::array<Heuristic*, NUM_QUEUES> heurs_array;
    for(int i=0; i < robot_heurs.size(); i++)
        heurs_array[i] = robot_heurs[i].get();

    std::vector<Heuristic*> heurs;
    for(int i=0; i < robot_heurs.size(); i++)
        heurs.push_back(robot_heurs[i].get());

    std::vector<smpl::RobotHeuristic*> all_robot_heurs;
    for(int i = 0; i < robot_heurs.size(); i++)
        all_robot_heurs.push_back(robot_heurs[i].get());

    Heuristic* anchor_heur = heurs[0];

    //if aij = 1 :  closed in rep i => closed in rep j
    std::array< std::array<int, NUM_ACTION_SPACES>, NUM_ACTION_SPACES >
        rep_dependency_matrix = {{ {{1, 1, 1}},
                                  {{0, 1, 0}},
                                  {{0, 0, 1}} }};

    std::vector<int> rep_ids_v;
    for(auto val : rep_ids)
        rep_ids_v.push_back(val);

    const unsigned int seed = 100;
    using PolicyT = RoundRobinPolicy;

    auto rr_policy = std::make_unique<PolicyT>(NUM_QUEUES - 1);

    using Planner = MRMHAPlannerMetaMHA<NUM_QUEUES, NUM_ACTION_SPACES, PolicyT>;
    auto search_ptr = std::make_unique<Planner>(
            space.get(), heurs_array, rep_ids, rep_dependency_matrix, rr_policy.get() );
    const int max_planning_time = planning_config.planning_time;
    const double eps = planning_config.eps;
    const double eps_mha = planning_config.eps_mha;
    MPlanner::PlannerParams planner_params = { max_planning_time, eps, eps_mha, false };

    using MotionPlanner = MPlanner::MotionPlanner<Planner, smpl::ManipLatticeMultiRep>;
    auto mplanner = std::make_unique<MotionPlanner>();
    mplanner->init(search_ptr.get(), space.get(), heurs, planner_params);

    MotionPlannerROS< Callbacks<>, StartGoalDatabase, MotionPlanner > mplanner_ros(ph, rm.get(), scene_ptr.get(), mplanner.get(), grid_ptr.get());

    // Add Start-goal pairs to the database:
    ReadExperimentsFromFile read_from_file(ph);
    PlanningEpisode ep = planning_config.start_planning_episode;
    mplanner_ros.setReferenceState(read_from_file.getStart(0));

    std::vector<smpl::RobotState> starts;
    std::vector<smpl::Affine3> goals;
    mplanner_ros.updateMap(0);
    while(ep <= std::min(planning_config.end_planning_episode, read_from_file.numEpisodes() - 1))
    {
        auto goal = read_from_file.getGoal(ep);
        space->setGoal(goal);
        for(int i = 0; i < space->numGoalBasePoses(); i++)
        {
            auto start = space->getGoalBasePose(i);
            std::stringstream ss;
            //for(int j = 0; j < start.size(); j++)
                //ss<< start[j]<< " ";
            //ROS_ERROR_STREAM(ss.str());
            if(cc.isStateValid(start, true))
                mplanner_ros.addStartGoal(start, goal);
            starts.push_back(start);
            goals.push_back(goal.pose);
        }
        ep++;
    }

    /*
    std::stringstream ss2;
    auto ref_state =  read_from_file.getStart(0);
    for(auto val : ref_state.joint_state.name)
    {
        ss2<< val<< " ";
        ROS_ERROR("%s", val.c_str());
    }
    writeStartGoals("train_start_states.txt", "train_goal_poses.txt", ss2.str(), starts, goals);
    */
    ROS_ERROR("Num starts: %d, goals: %d", mplanner_ros.m_start_states.size(), mplanner_ros.m_goal_constraints.size());

    std::string features_file_name = "train_features" + instance + ".txt";
    std::string delta_h_file_name = "train_delta_h" + instance + ".txt";
    std::ofstream features_file;
    std::ofstream delta_h_file;
    ExecutionStatus status = ExecutionStatus::WAITING;
    std::string file_prefix = "paths/solution_path";
    std::ofstream stats_file;
    ep = 0;

    ROS_WARN("Setting compute_base_poses flag to false");
    space->compute_base_poses = false;

    while(ep < mplanner_ros.numEpisodes())
    {
        ROS_ERROR("Episode: %d", ep);
        loop_rate.sleep();

        auto goal_trans = mplanner_ros.getGoal(ep).pose.translation();
        auto goal_xyprpy = poseToXYZRPY(mplanner_ros.getGoal(ep).pose);

        auto start = mplanner_ros.getStart(ep).joint_state.position;
        double ray_root[3];

        ray_root[0] = start[0];
        ray_root[1] = start[1];
        ray_root[2] = 1.1;

        double goal[3];
        goal[0] = goal_trans[0];
        goal[1] = goal_trans[1];
        goal[2] = goal_trans[2];

        //ROS_ERROR("Goal %f, %f, %f", goal[0], goal[1], goal[2]);
        //ROS_ERROR("Root %f, %f, %f", ray_root[0], ray_root[1], ray_root[2]);

        double dtheta = 10;
        double dr = 0.05;
        std::vector<double> ray_cast;
        grid_ptr->getRayCast(ray_root, goal, dtheta, dr, ray_cast);

        ROS_ERROR("Ray cast size: %d", ray_cast.size());
        // Ray-cast features
        features_file.open(features_file_name, std::ios::app);
        for(int i = 0; i < ray_cast.size(); i++)
            features_file<< ray_cast[i]<< " ";

        // Goal rpy
        features_file<< goal_xyprpy[3]<< " "<< goal_xyprpy[4]<< " "<< goal_xyprpy[5]<< " ";

        // Start yaw
        features_file<< start[2];
        features_file<< "\n";
        features_file.close();

        std::stringstream ss;
        for(auto val : ray_cast)
            ss<< val<< " ";
        ROS_ERROR_STREAM(ss.str());

        std::string file_suffix = std::to_string(ep) + ".txt";
        space->clearStates();
        space->clearStats();
        status = mplanner_ros.execute(ep);
        if(status == ExecutionStatus::SUCCESS)
        {
            ROS_INFO("----------------");
            ROS_INFO("Planning Time: %f", mplanner_ros.getPlan(ep).planning_time);
            ROS_INFO("----------------");
            auto plan = mplanner_ros.getPlan(ep).robot_states;

            // Write to file.
            stats_file.open("planning_stats.txt", std::ios::app);
            auto plan_stats = mplanner_ros.getPlan(ep);
            stats_file<<std::to_string(ep)<<" "<<max_planning_time<<" ";
            stats_file<<plan_stats.planning_time << " " << plan_stats.num_expansions << " " << plan_stats.cost<<" ";
            stats_file<<std::to_string(eps)<<" "<<std::to_string(eps_mha)<< " ";
            stats_file<< std::to_string(plan_stats.fullbody_expansions)<< " "<< std::to_string(plan_stats.base_expansions)<< " "<< std::to_string(plan_stats.arm_expansions)<< "\n";
            stats_file.close();

            std::string file_name = file_prefix + file_suffix;
            std::string header = "Solution Path";
            ROS_ERROR("%s", file_name.c_str());
            writePath(file_name, header , plan);
            //////////////

            visualization_msgs::MarkerArray whole_path;
            std::vector<visualization_msgs::Marker> m_all;

            int idx = 0;
            for( int pidx=0; pidx<plan.size(); pidx++ ){
                auto& state = plan[pidx];
                auto markers = cc.getCollisionRobotVisualization(state);
                for (auto& m : markers.markers) {
                    m.ns = "path_animation";
                    m.id = idx;
                    idx++;
                    whole_path.markers.push_back(m);
                }
                visualizer.visualize(smpl::visual::Level::Info, markers);
                //std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

        if(status != ExecutionStatus::WAITING)
        {
            delta_h_file.open(delta_h_file_name, std::ios::app);
            double delta_h = (1.0 * search_ptr->m_initial_h[1] - search_ptr->m_best_h[1]) / search_ptr->m_inad_expansions;
            ROS_ERROR("Delta h: %f", delta_h);
            delta_h_file<< delta_h<< "\n";
            delta_h_file.close();
        }

        if(status != ExecutionStatus::WAITING)
        {
            status = ExecutionStatus::WAITING;
            ep++;
        }
        ros::spinOnce();
    }
}
