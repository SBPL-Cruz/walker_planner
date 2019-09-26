//#include <vector>
//#include <string>
#include <sbpl_collision_checking/collision_space.h>
#include "planner_config.h"

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

std::vector<moveit_msgs::CollisionObject> GetMultiRoomMapCollisionCubes(
    const std::string& frame_id,
    const MultiRoomMapConfig config,
    std::vector<moveit_msgs::CollisionObject>&);

std::vector<moveit_msgs::CollisionObject> GetMultiRoomMapCollisionCubes(
    std::vector<moveit_msgs::CollisionObject>& doors,
    const std::string& frame_id,
    const double x_max,
    const double y_max,
    const double door_width,
    int n_tables);
