
#include "planner_config.h"

int main( int argc, char* argv[] ){
    ros::init(argc, argv, "interactive_walker_model");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    //ros::Rate loop_rate(10);

    std::string robot_description;
    std::strin planning_frame;
    nh.getParam("/robot_description", robot_description);
    RobotModelConfig robot_config;
    if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model"), robot_config)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return 1;
    }
    ph.getParam( "planning_frame", planning_frame );


    moveit_msgs::RobotState start_state;
    start_state.joint_state.position[0] = 0;
    start_state.joint_state.position[1] = 0;
    start_state.joint_state.position[2] = 0;

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
    grid.setReferenceFrame(planning_frame);

    //grid.addPointsToField(m_occgrid_points);
    SV_SHOW_INFO(grid.getBoundingBoxVisualization());

    //////////////////////////////////
    // Initialize Collision Checker //
    //////////////////////////////////

    ROS_INFO("Initializing collision checker");

    return 0;
}
