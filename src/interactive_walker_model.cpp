#include <interactive_markers/interactive_marker_server.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/ros/propagation_distance_field.h>
#include <smpl/debug/visualize.h>

#include <sbpl_collision_checking/collision_space.h>
#include "collision_space_scene.h"
#include "planner_config.h"

visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg ){
    visualization_msgs::Marker marker;
    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

  return marker;
}

visualization_msgs::InteractiveMarkerControl makeTransControl(){
    visualization_msgs::InteractiveMarkerControl trans_control;
    trans_control.always_visible = true;
    trans_control.markers.push_back(marker);
    trans_control.orientation.w = 1;
    trans_control.orientation.x = 0;
    trans_control.orientation.y = 1;
    trans_control.orientation.z = 0;
    trans_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

    return trans_control;
}

auto makeRotControl(){
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

    return control;
}

int main( int argc, char* argv[] ){
    ros::init(argc, argv, "interactive_walker_model");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    //ros::Rate loop_rate(10);

    std::string robot_description;
    std::string planning_frame;
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
    smpl::OccupancyGrid grid(df, ref_counted);
    grid.setReferenceFrame(planning_frame);

    //grid.addPointsToField(m_occgrid_points);
    SV_SHOW_INFO(grid.getBoundingBoxVisualization());

    //////////////////////////////////
    // Initialize Collision Checker //
    //////////////////////////////////

    ROS_INFO("Initializing collision checker");

    /////////////////////
    //Interactive Markers
    ////////////////////

    interactive_markers::InteractiveMarkerServer int_marker_server("interactive_walker_model");

    {//Start
        {//Start Base.
            visualization_msgs::InteractiveMarker robot_int_marker;
            robot_int_marker.header.frame_id = planning_frame;
            robot_int_marker.header.stamp = ros::Time::now();
            robot_int_marker.header.name = "Start Model Marker";

            auto trans_control = makeTransControl();
            auto rot_control = makeRotControl();
            robot_int_marker.controls.push_back( trans_control );
            robot_int_marker.controls.push_back( rot_control );

            int_marker_server.insert( robot_int_marker,
                    boost::bind(&ControlPlanner::processFeedback, this, _1) );

        }
        {//Start Right Hand
            visualization_msgs::InteractiveMarker robot_int_marker;
            robot_int_marker.header.frame_id = planning_frame;
            robot_int_marker.header.stamp = ros::Time::now();
            robot_int_marker.header.name = "Start Right Arm Marker";

            auto trans_control = makeTransControl();
            robot_int_marker.controls.push_back(trans_control);
            ->insert(int_marker, boost::bind(&ControlPlanner::processFeedback, this, _1));

        }
    }

    {//Goal Marker

    }

    return 0;
}
