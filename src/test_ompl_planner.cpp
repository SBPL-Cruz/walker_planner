#include <stdlib.h>
#include <iosfwd>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
// system includes
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <kdl_conversions/kdl_msg.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/RobotState.h>
#include <sbpl/planners/planner.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/spatial.h>
#include <smpl/ros/propagation_distance_field.h>
#include <sbpl_collision_checking/collision_space.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl_ompl_interface/ompl_interface.h>
#include <urdf_parser/urdf_parser.h>
#include <smpl_urdf_robot_model/robot_model.h>

//Local
#include "config/planner_config.h"
#include "config/collision_space_scene.h"
#include "config/get_collision_objects.h"
#include "motion_planner.h"
#include "motion_planner_ros.h"

//Publish Path
#include "walker_planner/GraspPose.h"
#include "walker_planner/Path1.h"

typedef ompl::base::SE2StateSpace::StateType SE2State;
typedef ompl::base::RealVectorStateSpace::StateType VectorState;

class ReadExperimentsFromFile {
    public:
    ReadExperimentsFromFile(ros::NodeHandle);
    moveit_msgs::RobotState getStart(PlanningEpisode);
    smpl::GoalConstraint getGoal(PlanningEpisode);
    std::vector<double> getFullbodyGoal(PlanningEpisode ep)
    {
        return m_goal_states[ep];
    }
    bool canCallPlanner() const { return true; }

    private:
    ros::NodeHandle m_nh;
    std::vector<moveit_msgs::RobotState> m_start_states;
    std::vector<smpl::GoalConstraint> m_goal_constraints;
    std::vector< smpl::RobotState> m_goal_states;

    moveit_msgs::RobotState stringToRobotState(std::string, std::string);
    bool init( std::string, std::string, std::string );
};

ReadExperimentsFromFile::ReadExperimentsFromFile(ros::NodeHandle _nh) :
    m_nh{_nh}{
    std::string start_file, goal_file, goal_pose_file;
    m_nh.getParam("robot_start_states_file", start_file);
    m_nh.getParam("robot_goal_poses_file", goal_pose_file);
    m_nh.getParam("robot_goal_states_file", goal_file);

    //ROS_ERROR("%s", start_file.c_str());
    //ROS_ERROR("%s", goal_pose_file.c_str());
    //ROS_ERROR("%s", goal_file.c_str());
    init(start_file, goal_pose_file, goal_file);
    ROS_ERROR("goal states: %d", m_goal_states.size());
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
    goal.rpy_tolerance[0] = 0.70;//0.39;
    goal.rpy_tolerance[1] = 0.70;//0.39;
    goal.rpy_tolerance[2] = 0.70;//0.39;

    return goal;
}

bool ReadExperimentsFromFile::init( std::string _start_file, std::string _goal_pose_file, std::string _goal_file ){
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

    {
        std::ifstream goal_stream;
        goal_stream.open(_goal_file);
        ROS_ERROR("%s", _goal_file.c_str());
        if(!goal_stream)
            throw "Could not read Start file.";
        std::string line_str;
        char line[100];
        while(goal_stream.getline(line, 100, '\n'))
        {
            std::string line_str(line);
            std::stringstream state_stream(line_str);
            std::string joint_val;
            smpl::RobotState robot_state;
            while(state_stream >> joint_val){
                robot_state.push_back(std::stod(joint_val));
            }
            m_goal_states.push_back(robot_state);
        }
        goal_stream.close();
    }


    std::ifstream goal_stream;
    goal_stream.open(_goal_pose_file);
    ROS_ERROR("%s", _goal_pose_file.c_str());
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


auto ConstructStateSpace(
    const urdf::ModelInterface& urdf,
    const std::vector<std::string>& planning_joints)
    -> ompl::base::StateSpacePtr
{
    auto* concrete_space = new ompl::base::CompoundStateSpace;

    ompl::base::SE2StateSpace* se2 = new ompl::base::SE2StateSpace();
    ompl::base::RealVectorBounds base_bounds(2);
    base_bounds.setLow(0, 0);
    base_bounds.setHigh(0, 6);
    base_bounds.setLow(1, 0);
    base_bounds.setHigh(1, 6);
    se2->setBounds(base_bounds);
    ompl::base::StateSpacePtr se2_ptr(se2);
    concrete_space->addSubspace(se2_ptr, 1);

    auto* subspace = new ompl::base::RealVectorStateSpace(7);
    ompl::base::RealVectorBounds bounds(7);

    for (int i = 3; i < planning_joints.size(); i++) {
        auto joint_name = planning_joints[i];
        auto joint = urdf.getJoint(joint_name);
        ROS_ERROR("Inside joint: %s", joint_name.c_str());
        bounds.setLow(i - 3, joint->limits->lower);
        bounds.setHigh(i - 3, std::min(joint->limits->upper, 6.30));
    }
    subspace->setBounds(bounds);

    ompl::base::StateSpacePtr subspace_ptr(subspace);
    concrete_space->addSubspace(subspace_ptr, 1.0);
    //ompl::base::StateSpacePtr concrete_space = se2_ptr + subspace_ptr;

    ROS_ERROR("Number of subspace: %d", concrete_space->getSubspaces().size());
    return ompl::base::StateSpacePtr(concrete_space);
}

struct ProjectionEvaluatorFK : public ompl::base::ProjectionEvaluator
{
    smpl::KDLRobotModel* model = NULL;
    const ompl::base::StateSpace* state_space = NULL;

    using Base = ompl::base::ProjectionEvaluator;

    ProjectionEvaluatorFK(const ompl::base::StateSpace* space);
    ProjectionEvaluatorFK(const ompl::base::StateSpacePtr& space);

    auto getDimension() const -> unsigned int override;
    void project(
            const ompl::base::State* state,
            ompl::base::EuclideanProjection& projection) const override;
    void setCellSizes(const std::vector<double>& cell_sizes) override;
    void defaultCellSizes() override;
    void setup() override;
    void printSettings(std::ostream& out = std::cout) const override;
    void printProjection(
            const ompl::base::EuclideanProjection& projection,
            std::ostream& out = std::cout) const override;
};

ProjectionEvaluatorFK::ProjectionEvaluatorFK(
    const ompl::base::StateSpace* space)
:
    Base(space)
{
    state_space = space;
}

ProjectionEvaluatorFK::ProjectionEvaluatorFK(
    const ompl::base::StateSpacePtr& space)
:
    Base(space)
{
    state_space = space.get();
}

auto ProjectionEvaluatorFK::getDimension() const -> unsigned int
{
    return 6;
}

void ProjectionEvaluatorFK::project(
        const ompl::base::State* state,
        ompl::base::EuclideanProjection& projection) const
{
    auto values = smpl::MakeStateSMPL(this->state_space, state);
    //for(int i =0; i <values.size(); i++)
        //std::cout<< values[i]<< " ";
    //std::cout<<"\n";
    auto pose = model->computeFK(values);
    projection.resize(getDimension(), 0.0);
    projection[0] = pose.translation().x();
    projection[1] = pose.translation().y();
    projection[2] = pose.translation().z();
    smpl::angles::get_euler_zyx(
            pose.rotation(), projection[3], projection[4], projection[5]);
}

void ProjectionEvaluatorFK::setCellSizes(const std::vector<double>& cell_sizes)
{
    this->Base::setCellSizes(cell_sizes);
}

void ProjectionEvaluatorFK::defaultCellSizes()
{
    this->Base::defaultCellSizes();
}

void ProjectionEvaluatorFK::setup()
{
    this->Base::setup();
}

void ProjectionEvaluatorFK::printSettings(std::ostream& out) const
{
    this->Base::printSettings(out);
}

void ProjectionEvaluatorFK::printProjection(
    const ompl::base::EuclideanProjection& projection, std::ostream& out) const
{
    this->Base::printProjection(projection, out);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_ompl_planner");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ROS_INFO("Initialize visualizer");
    smpl::VisualizerROS visualizer(nh, 100);
    smpl::viz::set_visualizer(&visualizer);

    // Let publishers set up
    ros::Duration(1.0).sleep();

    /////////////////
    // Robot Model //
    /////////////////

    ROS_INFO("Load common parameters");

    // Robot_description required to initialize collision checker, state space,
    // and forward kinematics...
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

    // Planning group required to initialize collision checker and state
    // space...
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

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    ROS_INFO("Initialize Occupancy Grid");

    auto df_size_x = 20.0;
    auto df_size_y = 15.0;
    auto df_size_z = 1.5;
    auto df_res = 0.05;
    auto df_origin_x = 0.0;
    auto df_origin_y = 0.0;
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

    /////////////////
    // Setup Scene //
    /////////////////

    ROS_INFO("Initialize scene");

    scene.SetCollisionSpace(&cc);

    // Update map
    std::string filename;
    ph.getParam("object_filename", filename);

    auto objects = GetCollisionObjects(filename, grid.getReferenceFrame());
    for (auto& object : objects)
        scene.ProcessCollisionObjectMsg(object);

    SV_SHOW_INFO(scene.getOccupiedVoxelsVisualization());

    //auto map_config = getMultiRoomMapConfig(ph);
    //std::vector<moveit_msgs::CollisionObject> tmp;
    //auto objects = GetMultiRoomMapCollisionCubes(grid.getReferenceFrame(), map_config, tmp);
    //for (auto& object : objects) {
        //scene.ProcessCollisionObjectMsg(object);
    //}

    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    SV_SHOW_INFO(grid.getOccupiedVoxelsVisualization());

    auto rm = SetupRobotModel(robot_description, robot_config);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

    ///////////////////
    // Planner Setup //
    ///////////////////

    ROS_INFO("Initialize the planner");

    // Construct state space from the urdf + planning group...
    auto urdf = urdf::parseURDF(robot_description);
    if (!urdf) {
        ROS_ERROR("Failed to parse URDF");
        return 1;
    }

    auto state_space = ConstructStateSpace(*urdf, robot_config.planning_joints);

    ompl::geometric::SimpleSetup ss(state_space);

    // Use CollisionSpace as the collision checker...
    ss.setStateValidityChecker([&](const ompl::base::State* state)
    {
        std::vector<double> values;// = state->reals();
        state_space->copyToReals(values, state);
        return cc.isStateValid(values);
    });

    // Set up a projection evaluator to provide forward kinematics...
    auto* fk_projection = new ProjectionEvaluatorFK(state_space);
    fk_projection->model = rm.get();
    state_space->registerProjection(
            "fk", ompl::base::ProjectionEvaluatorPtr(fk_projection));

    // Finally construct/initialize the planner...

    /*
    auto planner = std::make_unique(smpl::OMPLPlanner(
            ss.getSpaceInformation(), "arastar.bfs.manip", &grid));

    // Read params from the parameter server...
    PlannerConfig planning_config;
    if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)) {
        ROS_ERROR("Failed to read planner config");
        return 1;
    }

    // Set all planner params
    // TODO: handle discretization parameters
    planner->params().setParam("mprim_filename", planning_config.mprim_filename);
    planner->params().setParam("use_xyz_snap_mprim", planning_config.use_xyz_snap_mprim ? "1" : "0");
    planner->params().setParam("use_rpy_snap_mprim", planning_config.use_rpy_snap_mprim ? "1" : "0");
    planner->params().setParam("use_xyzrpy_snap_mprim", planning_config.use_xyzrpy_snap_mprim ? "1" : "0");
    planner->params().setParam("use_short_dist_mprims", planning_config.use_short_dist_mprims ? "1" : "0");
    planner->params().setParam("xyz_snap_dist_thresh", std::to_string(planning_config.xyz_snap_dist_thresh));
    planner->params().setParam("rpy_snap_dist_thresh", std::to_string(planning_config.rpy_snap_dist_thresh));
    planner->params().setParam("xyzrpy_snap_dist_thresh", std::to_string(planning_config.xyzrpy_snap_dist_thresh));
    planner->params().setParam("short_dist_mprims_thresh", std::to_string(planning_config.short_dist_mprims_thresh));
    planner->params().setParam("epsilon", "100.0");
    planner->params().setParam("search_mode", "0");
    planner->params().setParam("allow_partial_solutions", "0");
    planner->params().setParam("target_epsilon", "1.0");
    planner->params().setParam("delta_epsilon", "1.0");
    planner->params().setParam("improve_solution", "0");
    planner->params().setParam("bound_expansions", "1");
    planner->params().setParam("repair_time", "1.0");
    planner->params().setParam("bfs_inflation_radius", "0.04");
    planner->params().setParam("bfs_cost_per_cell", "100");

    planner->setStateVisualizer(
            [&](const std::vector<double>& state)
                -> std::vector<smpl::visual::Marker>
            {
                return cc.getCollisionModelVisualization(state);
            });

    ss.setPlanner(ompl::base::PlannerPtr(planner));
    */

    //////////////
    // Planning //
    //////////////

    ReadExperimentsFromFile read_exps_from_file(ph);

    int num_episodes = 10;
    for(int ep = 0; ep < num_episodes; ep++)
    {
        auto start = read_exps_from_file.getStart(ep);
        auto goal = read_exps_from_file.getGoal(ep);
        //auto goal = read_exps_from_file.getFullbodyGoal(ep);

        //Update Start
        auto marker = scene.getCollisionRobotVisualization(start.joint_state.position);
        for(auto& m : marker.markers)
            m.ns = "Start state";
        SV_SHOW_INFO(marker);
        smpl::urdf::RobotState reference_state;

        InitRobotState(&reference_state, &rm->m_robot_model);
        for (auto i = 0; i < start.joint_state.name.size(); ++i) {
            auto* var = GetVariable(&rm->m_robot_model, &start.joint_state.name[i]);
            if (var == NULL) {
                ROS_WARN("Failed to do the thing");
                continue;
            }
            ROS_INFO("Set joint %s to %f", start.joint_state.name[i].c_str(), start.joint_state.position[i]);
            SetVariablePosition(&reference_state, var, start.joint_state.position[i]);
        }
        SetReferenceState(rm.get(), GetVariablePositions(&reference_state));

        //ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(ss));

        // Update Goal

        // Set the goal state...
        Eigen::Affine3d goal_pose = goal.pose;
        for(int i=0; i<3; ++i)
            ROS_ERROR("Goal pose %d: %f", i, goal_pose.translation()[i]);

        auto goal_condition = std::make_shared<smpl::PoseGoal>(
                ss.getSpaceInformation(), goal_pose);
        goal_condition->position_tolerance = Eigen::Vector3d(goal.xyz_tolerance[0], goal.xyz_tolerance[1], goal.xyz_tolerance[2]);
        goal_condition->orientation_tolerance = Eigen::Vector3d(
                goal.rpy_tolerance[0],
                goal.rpy_tolerance[1],
                goal.rpy_tolerance[2]);
        ss.setGoal(ompl::base::GoalPtr(goal_condition));

        //ROS_ERROR("Goal set");

        //ompl::base::GoalState goal_state(ss.getSpaceInformation());
        //goal_state.setState();
        ////
        //ROS_ERROR("Starting start");
        //// Fullbody goal state
        //ompl::base::ScopedState<ompl::base::CompoundStateSpace> ompl_goal(state_space);
        //(*(ompl_goal->as<VectorState>(1)))[0] = goal[3];
        //(*(ompl_goal->as<VectorState>(1)))[1] = goal[4];
        //(*(ompl_goal->as<VectorState>(1)))[2] = goal[5];
        //(*(ompl_goal->as<VectorState>(1)))[3] = goal[6];
        //(*(ompl_goal->as<VectorState>(1)))[4] = goal[7];
        //(*(ompl_goal->as<VectorState>(1)))[5] = goal[8];
        //(*(ompl_goal->as<VectorState>(1)))[6] = goal[9];

        //ompl_goal->as<SE2State>(0)->setXY(goal[0],
                //goal[1]);
        //ompl_goal->as<SE2State>(0)->setYaw(goal[2]);

        //auto markers = cc.getCollisionRobotVisualization(goal);
        //int idxx = 0;
        //for (auto& m : markers.markers) {
            //m.ns = "goal_state";
            //m.id = idxx++;
        //}
        //visualizer.visualize(smpl::visual::Level::Info, markers);

        // Set the start state...
        ompl::base::ScopedState<ompl::base::CompoundStateSpace> ompl_start(state_space);
        (*(ompl_start->as<VectorState>(1)))[0] = start.joint_state.position[3];
        (*(ompl_start->as<VectorState>(1)))[1] = start.joint_state.position[4];
        (*(ompl_start->as<VectorState>(1)))[2] = start.joint_state.position[5];
        (*(ompl_start->as<VectorState>(1)))[3] = start.joint_state.position[6];
        (*(ompl_start->as<VectorState>(1)))[4] = start.joint_state.position[7];
        (*(ompl_start->as<VectorState>(1)))[5] = start.joint_state.position[8];
        (*(ompl_start->as<VectorState>(1)))[6] = start.joint_state.position[9];
        ompl_start->as<SE2State>(0)->setX(start.joint_state.position[0]);
        ompl_start->as<SE2State>(0)->setY(start.joint_state.position[1]);
        ompl_start->as<SE2State>(0)->setYaw(start.joint_state.position[2]);

        ss.setStartState(ompl_start);
        //pdef->setStartAndGoalStates(ompl_start, ompl_goal);
        std::cout<< ompl_start <<" \n";

        //ss.setStartAndGoalStates(ompl_start, ompl_goal);

        //ompl::base::ScopedState<ompl::base::SE2StateSpace> start_state1(state_space);
        //start_state1->setX(0.1);
        //start_state1->setY(0.2);
        //start_state1->setYaw(0.0);

        ROS_ERROR("start set");


        //OMPL Planner
        //auto pdef = std::make_shared<ompl::base::ProblemDefinition>(ss.getSpaceInformation());
        //pdef->addStartState(start.get());
        //pdef->setGoal(goal_condition);

        //ss.setup();
        // plan

        // XXX
        // RRT-Connect and BITstar need GoalSampleableRegion.
        // XXX
        //ompl::base::PlannerPtr planner(new ompl::geometric::BITstar(ss.getSpaceInformation()));
        //planner->as<ompl::geometric::SBL>()->setProjectionEvaluator("fk");
        ompl::base::PlannerPtr planner(new ompl::geometric::RRT(ss.getSpaceInformation()));
        //ompl::base::PlannerPtr planner(new ompl::geometric::RRTConnect(ss.getSpaceInformation()));
        dynamic_cast<ompl::geometric::RRT*>(planner.get())->setRange(0.1);
        //dynamic_cast<ompl::geometric::RRTConnect*>(planner.get())->setRange(0.1);
        ss.setPlanner(planner);

        ROS_INFO("Calling solve...");
        double allowed_planning_time;
        ph.param("allowed_planning_time", allowed_planning_time, 60.0);
        auto solved = ss.solve(allowed_planning_time);
        //if (solved != ompl::base::PlannerStatus::EXACT_SOLUTION || 
                //solved != ompl::base::PlannerStatus::APPROXIMATE_SOLUTION ) {
        if(!ss.haveSolutionPath()){
            ROS_INFO("Failed to plan.");
            return 0;
        }

        ///////////////////////////////////
        // Visualizations and Statistics //
        ///////////////////////////////////

        // TODO: print statistics

        std::cout << "Found solution of length " << ss.getSolutionPath().getStateCount() << "\n";
    //    ss.getSolutionPath().print(std::cout);

    //    ss.getSolutionPath().interpolate(50);
        ss.getSolutionPath().interpolate(100);
        std::cout << "Interpolated (raw) solution has " << ss.getSolutionPath().getStateCount() << " waypoints\n";
        //ss.simplifySolution();
        //std::cout << "Simplified solution has " << ss.getSolutionPath().getStateCount() << " waypoints\n";
        //ss.getSolutionPath().interpolate(1000);
        //std::cout << "Interpolated (simplified) solution has " << ss.getSolutionPath().getStateCount() << " waypoints\n";

        ROS_INFO("Animate path");

        size_t idx = 0;
        //while (ros::ok()) {
        for(int pidx = 0; pidx < ss.getSolutionPath().getStateCount(); pidx++){
            auto* state = ss.getSolutionPath().getState(pidx);
            auto point = smpl::MakeStateSMPL(state_space.get(), state);
            //for(auto val : point)
            //std::cout<< val <<" ";
            //std::cout<<"\n";
            //for(int i=0; i<point.size(); i++)
                //ROS_ERROR("%f", point[i]);
            auto markers = cc.getCollisionRobotVisualization(point);
            for (auto& m : markers.markers) {
                m.ns = "path_animation";
                m.id = idx++;
            }
            SV_SHOW_INFO(markers);
            visualizer.visualize(smpl::visual::Level::Info, markers);
            //std::this_thread::sleep_for(std::chrono::milliseconds(200));
            //std::this_thread::sleep_for(std::chrono::milliseconds(10));
            //pidx %= ss.getSolutionPath().getStateCount();
            //if (pidx == 0) break;
        }

    }

    return 0;
}
