#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <smpl/debug/visualizer_ros.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include "planner_config.h"
#include "start_goal_generator.h"
#include "collision_space_scene.h"
#include "get_collision_objects.h"

int  main(int argc, char** argv){
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

    auto objects = GetMultiRoomMapCollisionCubes(grid_ptr->getReferenceFrame(), 20, 15, .80, 1);
    for (auto& object : objects) {
        scene_ptr->ProcessCollisionObjectMsg(object);
    }

    ROS_INFO("Setting up robot model");
    auto rm = SetupRobotModel(robot_description, robot_config);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    StartGoalGenerator generator;
    generator.init(&cc, rm.get(), 1000);

    {
        BoundedRegion start_region;
        //std::vector<double> lo(rm->jointCount(), 0);
        std::vector<double> lo(12, 0);
        std::vector<double> hi(12, 0);
        lo[0] = 4.8;
        hi[0] = 5.2;
        lo[1] = 3.8;
        hi[1] = 4.2;
        lo[2] = -3.14;
        hi[2] = 3.14;
        ROS_ERROR("vprops Size: %d", rm->vprops.size());
        for(int i=5; i<10; i++){
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

        hi[0] = 0.5;
        lo[1] = 2;
        hi[1] = 3;

        lo[2] = 0.7;
        hi[2] = 0.7;
        for(int i=3; i<6; i++){
            lo[i] = -3.14;
            hi[i] = 3.14;
        }
        generator.addGoalRegion(goal_region);
    }

    auto status = generator.generate(10);
    ROS_ERROR("Status: %d", status);
    generator.writeToFile("start_states.txt", "goal_states.txt");
}

