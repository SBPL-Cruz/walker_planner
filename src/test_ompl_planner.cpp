#include <stdlib.h>
#include <iosfwd>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <smpl/angles.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/planning_params.h>
#include <smpl_ompl_interface/ompl_interface.h>
#include <urdf_parser/urdf_parser.h>

#include "config/planner_config.h"
#include "config/collision_space_scene.h"
#include "config/get_collision_objects.h"

auto ConstructStateSpace(
    const urdf::ModelInterface& urdf,
    const std::vector<std::string>& planning_joints)
    -> ompl::base::StateSpacePtr
{
    auto* concrete_space = new ompl::base::CompoundStateSpace;

    for (auto& joint_name : planning_joints) {
        auto joint = urdf.getJoint(joint_name);
        switch (joint->type) {
        case urdf::Joint::UNKNOWN:
            return NULL;
        case urdf::Joint::FIXED:
            break;
        case urdf::Joint::PRISMATIC:
        case urdf::Joint::REVOLUTE:
        {
            auto* subspace = new ompl::base::RealVectorStateSpace(1);
            if (joint->safety) {
                subspace->setBounds(joint->safety->soft_lower_limit, joint->safety->soft_upper_limit);
            } else if (joint->limits) {
                if(joint_name == "x" )
                    subspace->setBounds(0, 20);
                else if(joint_name == "y")
                    subspace->setBounds(0, 15);
                else
                subspace->setBounds(joint->limits->lower, std::min(joint->limits->upper, 20.0));
            } else {
                subspace->setBounds(-1.0, 1.0);
            }

            ompl::base::StateSpacePtr subspace_ptr(subspace);
            concrete_space->addSubspace(subspace_ptr, 1.0);
            break;
        }
        case urdf::Joint::CONTINUOUS:
        {
            concrete_space->addSubspace(
                    ompl::base::StateSpacePtr(new ompl::base::SO2StateSpace),
                    1.0);
            break;
        }
        case urdf::Joint::PLANAR:
        {
            auto* subspace = new ompl::base::SE2StateSpace;
            ompl::base::RealVectorBounds bounds(2);
            bounds.setLow(-1.0);
            bounds.setHigh(1.0);
            subspace->setBounds(bounds);
            concrete_space->addSubspace(ompl::base::StateSpacePtr(subspace), 1.0);
            break;
        }
        case urdf::Joint::FLOATING:
        {
            auto* subspace = new ompl::base::SE3StateSpace();
            ompl::base::RealVectorBounds bounds(3);
            bounds.setLow(-1.0);
            bounds.setHigh(1.0);
            subspace->setBounds(bounds);
            concrete_space->addSubspace(ompl::base::StateSpacePtr(subspace), 1.0);
            break;
        }
        default:
            ROS_WARN("Skip unrecognized joint type");
            break;
        }
    }

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
    auto map_config = getMultiRoomMapConfig(ph);
    std::vector<moveit_msgs::CollisionObject> tmp;
    auto objects = GetMultiRoomMapCollisionCubes(grid.getReferenceFrame(), map_config, tmp);
    for (auto& object : objects) {
        scene.ProcessCollisionObjectMsg(object);
    }

    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    SV_SHOW_INFO(grid.getOccupiedVoxelsVisualization());

    ros::Duration(1.0).sleep();

    auto rm = SetupRobotModel(robot_description, robot_config);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    // Read in start state from file and update the scene...
    // Start state is also required by the planner...
    moveit_msgs::RobotState start_state;
    if (!ReadInitialConfiguration(ph, start_state)) {
        ROS_ERROR("Failed to get initial configuration.");
        return 1;
    }

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

    // Set reference state in the collision model...
    SetReferenceState(rm.get(), GetVariablePositions(&reference_state));
    if (!scene.SetRobotState(start_state)) {
        ROS_ERROR("Failed to set start state on Collision Space Scene");
        return 1;
    }

    cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

    //SV_SHOW_INFO(grid.getDistanceFieldVisualization(0.2));

    //SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    //SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    //SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());

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

    ROS_INFO("Setup the query");


    // Set the goal state...
    Eigen::Affine3d goal_pose;
    {
        std::vector<double> goal(6, 0);
        ph.param("goal/x", goal[0], 0.0);
        ph.param("goal/y", goal[1], 0.0);
        ph.param("goal/z", goal[2], 0.0);
        ph.param("goal/yaw", goal[3], 0.0);
        ph.param("goal/pitch", goal[4], 0.0);
        ph.param("goal/roll", goal[5], 0.0);
        goal_pose =
                Eigen::Translation3d(goal[0], goal[1], goal[2]) *
                Eigen::AngleAxisd(goal[3], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(goal[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(goal[5], Eigen::Vector3d::UnitX());
    }
    for(int i=0; i<3; ++i)
        ROS_ERROR("Goal pose %d: %f", i, goal_pose.translation()[i]);

    auto goal_condition = std::make_shared<smpl::PoseGoal>(
            ss.getSpaceInformation(), goal_pose);
//    goal_condition->position_tolerance = Eigen::Vector3d(0.015, 0.015, 0.015);
    goal_condition->position_tolerance = Eigen::Vector3d(0.012, 0.012, 0.012);
    goal_condition->orientation_tolerance = Eigen::Vector3d(
            smpl::angles::to_radians(7.0),
            smpl::angles::to_radians(7.0),
            smpl::angles::to_radians(7.0));
    ss.setGoal(ompl::base::GoalPtr(goal_condition));

    //ss.setup();
    // Set the start state...
    auto found_valid_start = false;
    auto max_tries = 1;
    /*
    for (int i = 0; i < max_tries; ++i) {
        ROS_ERROR("Start try %d", i);
        ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(state_space);
        start.random();
        std::cout<<"Start:\n";
        std::cout<<start<<"\n";
        std::vector<double> values = start.reals();
        ROS_ERROR("Copied");
        ROS_ERROR("Copied size: %d", values.size());
        //for(auto& val : values)
            //ROS_ERROR("%d", values[i]);
        if (ss.getspaceinformation()->spaceinformation::isvalid(start.get())) {
            ss.setstartstate(start);
            found_valid_start = true;
            break;
        }
    }
    */
    std::vector<double> start_vals(10, 0);
    start_vals[0] = 5;
    start_vals[1] = 4;
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(state_space);
    start = start_vals;
    //if (ss.getSpaceInformation()->SpaceInformation::isValid(start.get())) {
        ss.setStartState(start);
        found_valid_start = true;
    //}

    if (!found_valid_start) {
        //ROS_WARN("Failed to find valid start state after %d tries", max_tries);
    } else{
        ROS_INFO("Found valid start.");
    }

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
    dynamic_cast<ompl::geometric::RRT*>(planner.get())->setRange(0.1);
    ss.setPlanner(planner);

    ROS_INFO("Calling solve...");
    double allowed_planning_time;
    ph.param("allowed_planning_time", allowed_planning_time, 10.0);
    auto solved = ss.solve(allowed_planning_time);
    if (!solved) {
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
    ss.getSolutionPath().interpolate(1000);
    std::cout << "Interpolated (raw) solution has " << ss.getSolutionPath().getStateCount() << " waypoints\n";
    ss.simplifySolution();
    std::cout << "Simplified solution has " << ss.getSolutionPath().getStateCount() << " waypoints\n";
    ss.getSolutionPath().interpolate(1000);
    std::cout << "Interpolated (simplified) solution has " << ss.getSolutionPath().getStateCount() << " waypoints\n";

    ROS_INFO("Animate path");

    size_t pidx = 0;
    while (ros::ok()) {
        auto* state = ss.getSolutionPath().getState(pidx);
        auto point = smpl::MakeStateSMPL(state_space.get(), state);
        //for(int i=0; i<point.size(); i++)
            //ROS_ERROR("%f", point[i]);
        auto markers = cc.getCollisionRobotVisualization(point);
        for (auto& m : markers.markers) {
            m.ns = "path_animation";
        }
        SV_SHOW_INFO(markers);
//        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        pidx++;
        pidx %= ss.getSolutionPath().getStateCount();
        if (pidx == 0) break;
    }

    return 0;
}
