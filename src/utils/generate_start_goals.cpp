#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <smpl/debug/visualizer_ros.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <trac_ik_robot_model/trac_ik_robot_model.h>

#include "config/planner_config.h"
#include "utils/start_goal_generator.h"
#include "config/collision_space_scene.h"
#include "config/get_collision_objects.h"

//using RobotModel = smpl::TracIKRobotModel;
using RobotModel = smpl::KDLRobotModel;

bool addStartGoalRegionsForDoor(
        StartGoalGenerator<RobotModel>& generator,
        const smpl::urdf::URDFRobotModel* rm,
        const std::vector<moveit_msgs::CollisionObject>& doors){
    double door_x = doors[0].primitive_poses[0].position.x;
    double door_y = doors[0].primitive_poses[0].position.y;

    {
        BoundedRegion start_region;
        //std::vector<double> lo(rm->jointCount(), 0);
        std::vector<double> lo(10, 0);
        std::vector<double> hi(10, 0);
        lo[0] = door_x - 0.5;
        hi[0] = door_x + 0.5;
        lo[1] = door_y - 1.3;
        hi[1] = door_y - 0.3;
        lo[2] = -3.14;
        hi[2] = 3.14;
        for(int i=3; i<10; i++){
            hi[i] = rm->vprops[i].max_position;
            lo[i] = rm->vprops[i].min_position;
            ROS_INFO("%f, %f", lo[i], hi[i]);
        }
        start_region.lo = lo;
        start_region.hi = hi;
        generator.addStartRegion(start_region);
    }

    {
        BoundedRegion goal_region;

        std::vector<double> lo(6, 0), hi(6, 0);

        lo[0] = door_x - 0.5;
        hi[0] = door_x + 0.5;
        lo[1] = door_y + 1;
        hi[1] = door_y + 1.5;
        lo[2] = 0.7;
        hi[2] = 0.8;

        lo[3] = -3.14;
        hi[3] = 3.14;
        for(int i=4; i<6; i++){
            lo[i] = -1.57;
            hi[i] = 1.57;
        }
        goal_region.lo = lo;
        goal_region.hi = hi;
        generator.addGoalRegion(goal_region);
    }
    return true;
}

bool addGoalRegionsForTable(
        StartGoalGenerator<RobotModel>& generator,
        const smpl::urdf::URDFRobotModel* rm,
        const std::vector<moveit_msgs::CollisionObject>& tables){

    for(auto& table : tables ){
        BoundedRegion goal_region;
        std::vector<double> lo(6, 0), hi(6, 0);

        double table_x = table.primitive_poses[0].position.x;
        double table_y = table.primitive_poses[0].position.y;
        double table_size_x = table.primitives[0].dimensions[0];
        double table_size_y = table.primitives[0].dimensions[1];
        double table_height = table.primitives[0].dimensions[2];

        lo[0] = table_x - table_size_x/2;
        hi[0] = table_x + table_size_x/2;
        lo[1] = table_y - table_size_y/2;
        hi[1] = table_y + table_size_y/2;
        lo[2] = table_height + 0.05;
        hi[2] = table_height + 0.18;

        //roll
        lo[3] = -3.14;
        hi[3] = 3.14;

        // Pitch
        lo[4] = -3.14; //0
        hi[4] = 3.14; //-0.1

        //yaw all around
        lo[5] = -3.14;
        hi[5] = 3.14;

        goal_region.lo = lo;
        goal_region.hi = hi;
        generator.addGoalRegion(goal_region);
    }
    return true;
}

// The Easy tests
bool addStartRegionsForRoom1(
        StartGoalGenerator<RobotModel>& generator,
        const smpl::urdf::URDFRobotModel* rm,
        double map_x,
        double map_y){

    {
        BoundedRegion start_region;
        std::vector<double> lo(10, 0);
        std::vector<double> hi(10, 0);
        lo[0] = 0.5;
        hi[0] = map_x/2 - 0.5;
        lo[1] = 0.5;
        hi[1] = map_y/3 - 0.5;
        lo[2] = -3.14;
        hi[2] = 3.14;
        for(int i=3; i<10; i++){
            hi[i] = rm->vprops[i].max_position;
            lo[i] = rm->vprops[i].min_position;
            ROS_INFO("%f, %f", lo[i], hi[i]);
        }
        start_region.lo = lo;
        start_region.hi = hi;
        generator.addStartRegion(start_region);
    }
    return true;
}

bool addStartRegionsForRoom3(
        StartGoalGenerator<RobotModel>& generator,
        const smpl::urdf::URDFRobotModel* rm,
        double map_x,
        double map_y){

    {
        BoundedRegion start_region;
        std::vector<double> lo(10, 0);
        std::vector<double> hi(10, 0);
        lo[0] = 0.5;
        hi[0] = map_x/2 - 0.5;
        lo[1] = map_y/3 + 0.5;
        hi[1] = 2*map_y/3 - 0.5;
        lo[2] = -3.14;
        hi[2] = 3.14;
        for(int i=3; i<10; i++){
            hi[i] = rm->vprops[i].max_position;
            lo[i] = rm->vprops[i].min_position;
            ROS_INFO("%f, %f", lo[i], hi[i]);
        }
        start_region.lo = lo;
        start_region.hi = hi;
        generator.addStartRegion(start_region);
    }
    return true;
}


bool addStartRegionsForRoom4(
        StartGoalGenerator<RobotModel>& generator,
        const smpl::urdf::URDFRobotModel* rm,
        double map_x,
        double map_y){

    {
        BoundedRegion start_region;
        std::vector<double> lo(10, 0);
        std::vector<double> hi(10, 0);
        lo[0] = map_x/2 + 0.5;
        hi[0] = map_x - 0.5;
        lo[1] = map_y/2 + 0.5;
        hi[1] = map_y - 0.5;
        lo[2] = -3.14;
        hi[2] = 3.14;
        for(int i=3; i<10; i++){
            hi[i] = rm->vprops[i].max_position;
            lo[i] = rm->vprops[i].min_position;
            ROS_INFO("%f, %f", lo[i], hi[i]);
        }
        start_region.lo = lo;
        start_region.hi = hi;
        generator.addStartRegion(start_region);
    }
    return true;
}

// The Hard tests
bool addStartRegionsForRoom5(
        StartGoalGenerator<RobotModel>& generator,
        const smpl::urdf::URDFRobotModel* rm,
        double map_x,
        double map_y){

    {
        BoundedRegion start_region;
        std::vector<double> lo(10, 0);
        std::vector<double> hi(10, 0);
        lo[0] = 0.5;
        hi[0] = map_x/2 - 0.5;
        lo[1] = 2*map_y/3 + 0.5;
        hi[1] = map_y - 0.5;
        lo[2] = -3.14;
        hi[2] = 3.14;
        for(int i=3; i<10; i++){
            hi[i] = rm->vprops[i].max_position;
            lo[i] = rm->vprops[i].min_position;
            ROS_INFO("%f, %f", lo[i], hi[i]);
        }
        start_region.lo = lo;
        start_region.hi = hi;
        generator.addStartRegion(start_region);
    }
    return true;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "generate_start_goal");
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

    std::vector<moveit_msgs::CollisionObject> doors;
    auto map_config = getMultiRoomMapConfig(ph);
    //auto objects = GetMultiRoomMapCollisionCubes(doors, grid_ptr->getReferenceFrame(), map_config);
    auto objects = GetMultiRoomMapCollisionCubes( grid_ptr->getReferenceFrame(), map_config, doors );
    for (auto& object : objects) {
        scene_ptr->ProcessCollisionObjectMsg(object);
    }

    ROS_INFO("Setting up robot model");
    auto fullbody_rm = SetupRobotModel<RobotModel>(robot_description, robot_config);
    //auto arm_rm = std::make_unique<RobotModel>();
    //if(!arm_rm->init(robot_description, "base_link", robot_config.chain_tip_link, 3)){
        //ROS_ERROR("Failed to initialize RobotModel");
        //return 1;
    //}

    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());

    StartGoalGenerator<RobotModel> generator;

    //Get table objects.
    std::vector<moveit_msgs::CollisionObject> tables;
    for(auto& obj : objects){
        ROS_ERROR("%s", obj.id.c_str());
        if(obj.id.compare( 0, 5, "table" ) == 0){
            tables.push_back(obj);
        }
    }
    ROS_INFO("%d tables found in map.", tables.size());
    generator.init(&cc, fullbody_rm.get(), 1000);
    //addStartGoalRegionsForDoor(generator, rm.get(), doors);
    addStartRegionsForRoom1(generator, fullbody_rm.get(), map_config.x_max, map_config.y_max);
    addStartRegionsForRoom3(generator, fullbody_rm.get(), map_config.x_max, map_config.y_max);
    addStartRegionsForRoom4(generator, fullbody_rm.get(), map_config.x_max, map_config.y_max);
    addStartRegionsForRoom5(generator, fullbody_rm.get(), map_config.x_max, map_config.y_max);
    addGoalRegionsForTable(generator, fullbody_rm.get(), tables);

    const int N = 500;
    auto status = generator.generate(N);
    if(status)
        ROS_INFO("Generated %d start-goal pairs.", N);
    else
        ROS_ERROR("Could not generate start-goal pairs.");
    generator.writeToFile(
            "x y theta right_j1 right_j2 right_j3 right_j4 right_j5 right_j6 right_j7\n",
            "start_states.txt", "goal_states.txt", "goal_poses.txt");
}

