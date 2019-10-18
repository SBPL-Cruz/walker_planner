#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <memory>
#include <fstream>
#include <stdlib.h>
#include <time.h>

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

#include "planners/mrmhaplanner.h"
#include "planners/mhaplanner.h"
#include "motion_planner.h"
#include "motion_planner_ros.h"
#include "scheduling_policies.h"
#include "utils/utils.h"

#define NUM_QUEUES 10
#define NUM_ACTION_SPACES 3

enum ActionSpace {
    Fullbody = 0,
    Arm,
    Base
};

bool constructHeuristics(
        std::vector<std::unique_ptr<smpl::RobotHeuristic>>& heurs,
        smpl::ManipLattice* pspace,
        smpl::OccupancyGrid* grid,
        smpl::KDLRobotModel* rm,
        PlannerConfig& params ){

    SMPL_INFO("Initialize Heuristics");
    const int DefaultCostMultiplier = 1000;

    struct AnchorHeuristic : public BfsHeuristic {
        int GetGoalHeuristic(int state_id) override {
            return std::max(bfs_3d_base->GetGoalHeuristic(state_id), bfs_3d->GetGoalHeuristic(state_id));
        }

        double getMetricGoalDistance(double x, double y, double z) override {
            return bfs_3d->getMetricGoalDistance(x, y, z);
        }

        double getMetricStartDistance(double x, double y, double z) override {
            return bfs_3d->getMetricStartDistance(x, y, z);
        }
    };

    struct EndEffHeuristic : public BfsHeuristic {
        bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
                std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d){
            if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
                return false;
            pose_ext = bfs_3d->planningSpace()->getExtension<smpl::PoseProjectionExtension>();
            return true;
        }

        int GetGoalHeuristic(int state_id){
            if (state_id == bfs_3d->planningSpace()->getGoalStateID()) {
                return 0;
            }
            if(pose_ext == nullptr)
                return 0;
            smpl::Affine3 p;
            if(!pose_ext->projectToPose(state_id, p))
                return 0;

            auto goal_pose = bfs_3d->planningSpace()->goal().pose;

            smpl::Quaternion qa(p.rotation());
            smpl::Quaternion qb(goal_pose.rotation());
            double dot = qa.dot(qb);
            if (dot < 0.0) {
                qb = smpl::Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
                dot = qa.dot(qb);
            }
            int rot_dist = DefaultCostMultiplier*smpl::angles::normalize_angle(2.0 * std::acos(dot));

            int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
            int arm_dist = bfs_3d->GetGoalHeuristic(state_id);

            int heuristic = base_coeff*base_dist + arm_coeff*arm_dist + rot_coeff*rot_dist;
            return heuristic;
        }

        double base_coeff=0.2;
        double arm_coeff=10;
        double rot_coeff=10;
        smpl::PoseProjectionExtension* pose_ext = nullptr;
    };

    struct RetractArmHeuristic : public BfsHeuristic {
        bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
                std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d){
            if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
                return false;
            return true;
        }

        int GetGoalHeuristic(int state_id){
            if(state_id == 0)
                return 0;

            smpl::Vector3 p;
            if(!bfs_3d->m_pp->projectToPoint(state_id, p)){
                ROS_ERROR("RetractArmHeuristic Could not project");
                return 0;
            }
            auto retracted_robot_state = bfs_3d->planningSpace()->getExtension<smpl::ExtractRobotStateExtension>()->extractState(state_id);

            // Norm of first 5 joints.
            double norm = 0.0;
            for(int i=3; i<8; i++)
                norm += (retracted_robot_state[i]*retracted_robot_state[i]);
            norm = sqrt(norm);
            int retract_heuristic = DefaultCostMultiplier * norm;

            int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);

            int heuristic = base_coeff*base_dist + retract_arm_coeff*retract_heuristic;

            return heuristic;
        }

        double base_coeff = 0.0;
        double retract_arm_coeff = 10.0;

    };

    struct ImprovedEndEffHeuristic : public BfsHeuristic {
        bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
                std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d,
                std::shared_ptr<RetractArmHeuristic> _retract_arm){
            if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
                return false;
            m_retract_arm_heur = _retract_arm;
            pose_ext = bfs_3d->planningSpace()->getExtension<smpl::PoseProjectionExtension>();
            return true;
        }

        int GetGoalHeuristic(int state_id){
            if (state_id == bfs_3d->planningSpace()->getGoalStateID()) {
                return 0;
            }
            if(pose_ext == nullptr)
                return 0;
            smpl::Affine3 p;
            if(!pose_ext->projectToPose(state_id, p))
                return 0;

            auto goal_pose = bfs_3d->planningSpace()->goal().pose;

            smpl::Quaternion qa(p.rotation());
            smpl::Quaternion qb(goal_pose.rotation());
            double dot = qa.dot(qb);
            if (dot < 0.0) {
                qb = smpl::Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
                dot = qa.dot(qb);
            }
            int rot_dist = DefaultCostMultiplier*smpl::angles::normalize_angle(2.0 * std::acos(dot));

            int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
            int arm_dist  = 0;
            if(arm_dist > 10000){
                arm_dist = m_retract_arm_heur->GetGoalHeuristic(state_id);
                rot_dist = 0.0;
            } else {
                arm_dist = bfs_3d->GetGoalHeuristic(state_id);
            }

            int heuristic = base_coeff*base_dist + arm_coeff*arm_dist + rot_coeff*rot_dist;
            return heuristic;
        }

        double base_coeff=0.05;
        double arm_coeff=1;
        double rot_coeff=2;

        std::shared_ptr<RetractArmHeuristic> m_retract_arm_heur;
        smpl::PoseProjectionExtension* pose_ext = nullptr;
    };

    struct BaseRotHeuristic : public BfsHeuristic {
        bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
                std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d,
                std::shared_ptr<RetractArmHeuristic> _retract_arm,
                double _orientation){
            if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
                return false;
            m_retract_arm_heur = _retract_arm;
            orientation = _orientation;
            return true;
        }

        int GetGoalHeuristic(int state_id){
            if(state_id == 0)
                return 0;
            auto robot_state = (dynamic_cast<smpl::ManipLattice*>(
                        bfs_3d->planningSpace()))->extractState(state_id);
            int yaw_dist = DefaultCostMultiplier*smpl::angles::shortest_angle_dist(robot_state[2], orientation);

            int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
            double arm_fold_heur = 0.0;
            if(base_dist > 10000)
                arm_fold_heur = m_retract_arm_heur->GetGoalHeuristic(state_id);

            int heuristic = base_dist + orientation_coeff*yaw_dist + arm_fold_coeff*arm_fold_heur;
            //ROS_ERROR("%d + %d + %d = %d", int(base_dist), int(orientation_coeff*yaw_dist), int(arm_fold_coeff*arm_fold_heur), heuristic);
            return heuristic;
        }

        std::shared_ptr<RetractArmHeuristic> m_retract_arm_heur;
        double orientation = 0.0;
        double orientation_coeff = 5.0;
        double arm_fold_coeff = 1.0;
    };


    auto bfs_2d = std::make_unique<smpl::Bfs2DHeuristic>();
    bfs_2d->setCostPerCell(params.cost_per_cell);
    bfs_2d->setInflationRadius(params.inflation_radius_2d);
    if (!bfs_2d->init(pspace, grid)) {
        ROS_ERROR("Could not initialize Bfs2Dheuristic.");
        return false;
    }

    auto bfs_3d = std::make_shared<smpl::Bfs3DHeuristic>();
    bfs_3d->setCostPerCell(params.cost_per_cell);
    bfs_3d->setInflationRadius(params.inflation_radius_3d);
    if (!bfs_3d->init(pspace, grid)) {
        ROS_ERROR("Could not initialize Bfs3Dheuristic.");
        return false;
    }

    auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
    bfs_3d_base->setCostPerCell(params.cost_per_cell);
    bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
    if (!bfs_3d_base->init(pspace, grid, 4)) {
        ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
        return false;
    }

    auto retract_arm = std::make_shared<RetractArmHeuristic>();
    if(!retract_arm->init(bfs_3d_base, bfs_3d)){
        ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
        return false;
    }

    //Compute a feasible base location.
    std::vector<int> base_x, base_y;
    heurs.clear();

    {
        auto anchor = std::make_unique<AnchorHeuristic>();
        anchor->init( bfs_3d_base, bfs_3d );
        heurs.push_back(std::move(anchor));
    }
    { //EndEffector
        auto inad = std::make_unique<ImprovedEndEffHeuristic>();
        inad->init( bfs_3d_base, bfs_3d, retract_arm );
        heurs.push_back(std::move(inad));
    }

    int num_rot_heurs = 8;
    for(int i=0; i<num_rot_heurs; i++){
        auto h = std::make_unique<BaseRotHeuristic>();
        if (!h->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        heurs.push_back(std::move(h));
    }

    for (auto& entry : heurs) {
        pspace->insertHeuristic(entry.get());
    }
    return true;
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

int main(int argc, char** argv) {
    SMPL_INFO("Testing the new templated MRMHAPlanner.");
    ros::init(argc, argv, "test_templated_mrmha_planner");
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

    ///////////////
    //Planning////
    /////////////

    std::vector< std::unique_ptr<smpl::RobotHeuristic> > robot_heurs;

    if(!constructHeuristics( robot_heurs, space.get(), grid_ptr.get(), rm.get(), planning_config )){
        ROS_ERROR("Could not construct heuristics.");
        return 0;
    }

    ROS_ERROR("Number of heuristics: %d", robot_heurs.size());
    assert(robot_heurs.size() == NUM_QUEUES);

    assert(robot_heurs[0] != nullptr);

    std::array<Heuristic*, NUM_QUEUES> heurs_array;
    for(int i=0; i < robot_heurs.size(); i++)
        heurs_array[i] = robot_heurs[i].get();

    std::vector<Heuristic*> heurs;
    for(int i=0; i < robot_heurs.size(); i++)
        heurs.push_back(robot_heurs[i].get());

    Heuristic* anchor_heur = heurs[0];
    std::vector<Heuristic*> inad_heurs( heurs.begin() + 1, heurs.end() );

    std::array<int, NUM_QUEUES> rep_ids;
    for(auto& ele : rep_ids)
        ele = (int)Base;
    rep_ids[0] = (int)Fullbody;
    rep_ids[1] = (int)Fullbody;

    // if aij = 1 :  closed in rep i => closed in rep j
    std::array< std::array<int, NUM_ACTION_SPACES>, NUM_ACTION_SPACES >
        rep_dependency_matrix = {{ {{1, 1, 1}},
                                  {{0, 1, 0}},
                                  {{0, 1, 0}} }};

    //auto uniformly_random_policy = std::make_unique<UniformlyRandomPolicy>(inad_heurs.size(), 100);
    auto round_robin_policy = std::make_unique<RoundRobinPolicy>(inad_heurs.size());

    //using Planner = MRMHAPlanner<NUM_QUEUES, NUM_ACTION_SPACES, UniformlyRandomPolicy>;
    using Planner = MRMHAPlanner<NUM_QUEUES, NUM_ACTION_SPACES, RoundRobinPolicy>;
    //using Planner = MHAPlanner<NUM_QUEUES, RoundRobinPolicy>;
    //auto search_ptr = std::make_unique<Planner>(
    //        space.get(), heurs_array, rep_ids, rep_dependency_matrix, uniformly_random_policy.get() );
    auto search_ptr = std::make_unique<Planner>(
            space.get(), heurs_array, rep_ids, rep_dependency_matrix, round_robin_policy.get() );
    //auto search_ptr = std::make_unique<Planner>(space.get(), heurs_array, round_robin_policy.get());

    const int max_planning_time = planning_config.planning_time;
    const double eps = planning_config.eps;
    const double eps_mha = planning_config.eps_mha;
    MPlanner::PlannerParams planner_params = { max_planning_time, eps, eps_mha, false };

    //using MotionPlanner = MPlanner::MotionPlanner<Planner, smpl::ManipLatticeMultiRep>;
    using MotionPlanner = MPlanner::MotionPlanner<Planner, smpl::ManipLattice>;
    auto mplanner = std::make_unique<MotionPlanner>();
    mplanner->init(search_ptr.get(), space.get(), heurs, planner_params);

    MotionPlannerROS< Callbacks, ReadExperimentsFromFile, MotionPlanner > mplanner_ros(ph, rm.get(), scene_ptr.get(), mplanner.get(), grid_ptr.get());

    ExecutionStatus status = ExecutionStatus::WAITING;
    //while(status == ExecutionStatus::WAITING) {
    std::string file_prefix = "paths/solution_path";
    std::ofstream stats_file;
    PlanningEpisode ep = planning_config.start_planning_episode;
    while(ep <= planning_config.end_planning_episode){
        ROS_ERROR("Episode: %d", ep);
        loop_rate.sleep();
        std::string file_suffix = std::to_string(ep) + ".txt";
        status = mplanner_ros.execute(ep);
    //}
        if(status == ExecutionStatus::SUCCESS){
            ROS_INFO("----------------");
            ROS_INFO("Planning Time: %f", mplanner_ros.getPlan(ep).planning_time);
            ROS_INFO("----------------");
            auto plan = mplanner_ros.getPlan(ep).robot_states;

            // Write to file.
            stats_file.open("planning_stats.txt", std::ios::app);
            auto plan_stats = mplanner_ros.getPlan(ep);
            stats_file<<std::to_string(ep)<<" "<<max_planning_time<<" ";
            stats_file<<plan_stats.planning_time << " " << plan_stats.num_expansions << " " << plan_stats.cost<<" ";
            stats_file<<std::to_string(eps)<<" "<<std::to_string(eps_mha)<<"\n";
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

        if(status != ExecutionStatus::WAITING){
            status = ExecutionStatus::WAITING;
            ep++;
        }
        ros::spinOnce();
    }
}
