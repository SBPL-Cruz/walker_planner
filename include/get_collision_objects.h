//#include <vector>
//#include <string>
#include <sbpl_collision_checking/collision_space.h>

moveit_msgs::CollisionObject GetCollisionCube(
    const geometry_msgs::Pose& pose,
    std::vector<double>& dims,
    const std::string& frame_id,
    const std::string& id);

std::vector<moveit_msgs::CollisionObject> GetCollisionCubes(
    std::vector<std::vector<double>>& objects,
    std::vector<std::string>& object_ids,
    const std::string& frame_id);

std::vector<moveit_msgs::CollisionObject> GetCollisionObjects(
    const std::string& filename,
    const std::string& frame_id);

struct MultiRoomMapConfig {
    int seed = 1000;
    //Map
    double x_max = 15;
    double y_max = 15;
    double h_max = 1.5;
    double door_width = 1.0;
    double alley_width = 2.0;
    //Tables
    int n_tables = 0;
    double min_table_len = 0.5;
    double max_table_len = 1.0;
    double table_height = 0.6;
    double min_dist_bw_tables = 1.5;
};

std::vector<moveit_msgs::CollisionObject> GetMultiRoomMapCollisionCubes(
    const std::string& frame_id,
    const MultiRoomMapConfig config);
std::vector<moveit_msgs::CollisionObject> GetMultiRoomMapCollisionCubes(
    const std::string& frame_id,
    const double x_max,
    const double y_max,
    const double door_width,
    int n_tables);
