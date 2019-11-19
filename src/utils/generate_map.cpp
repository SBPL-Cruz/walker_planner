#include <ros/ros.h>
#include <vector>
#include <string>
#include <fstream>

#include <smpl/debug/visualizer_ros.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include "config/get_collision_objects.h"
#include "config/planner_config.h"

bool writeCollisionCubesToFile(
        std::vector<moveit_msgs::CollisionObject>& _cubes,
        std::string _file_name){
    std::ofstream file_stream;
    file_stream.open(_file_name);
    if(!file_stream.is_open()){
        ROS_ERROR("Could not open file.");
        return false;
    }

    file_stream<< _cubes.size() << "\n";
    for(const auto& cube : _cubes){
        auto pose = cube.primitive_poses[0];
        auto dims = cube.primitives[0].dimensions;
        file_stream<< cube.id<< " ";
        file_stream<< pose.position.x<< " ";
        file_stream<< pose.position.y<< " ";
        file_stream<< pose.position.z<< " ";
        file_stream<< dims[0]<< " ";
        file_stream<< dims[1]<< " ";
        file_stream<< dims[2];
        file_stream<< "\n";
    }
    file_stream.close();
    return true;
}

int main( int argc, char** argv ){
    ros::init(argc, argv, "generate_map");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Rate loop_rate(10);
    smpl::VisualizerROS visualizer(nh, 100);
    smpl::viz::set_visualizer(&visualizer);

    //auto file_name = argv[1];
    auto file_name = "multi_room_map.env";

    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

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
    std::vector<moveit_msgs::CollisionObject> doors;
    auto map_config = getMultiRoomMapConfig(ph);
    auto objects = GetMultiRoomMapCollisionCubes( grid_ptr->getReferenceFrame(), map_config, doors );
    writeCollisionCubesToFile(objects, file_name);
}
