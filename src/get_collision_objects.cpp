#include "get_collision_objects.h"

moveit_msgs::CollisionObject GetCollisionCube(
    const geometry_msgs::Pose& pose,
    std::vector<double>& dims,
    const std::string& frame_id,
    const std::string& id) {
    moveit_msgs::CollisionObject object;
    object.id = id;
    if( id.compare( 0, 4, "door" ) == 0 ){
        ROS_ERROR("%s", id.c_str());
        object.operation = moveit_msgs::CollisionObject::REMOVE;
    }
    else
        object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box_object;
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

std::vector<moveit_msgs::CollisionObject> GetCollisionCubes(
    std::vector<std::vector<double>>& objects,
    std::vector<std::string>& object_ids,
    const std::string& frame_id)
{
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3,0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        ROS_INFO("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(GetCollisionCube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

std::vector<moveit_msgs::CollisionObject> GetCollisionObjects(
    const std::string& filename,
    const std::string& frame_id) {
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double> > objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    FILE* fCfg = fopen(filename.c_str(), "r");

    if (fCfg == NULL) {
        ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg,"%s",sTemp) < 1) {
        printf("Parsed string has length < 1.\n");
    }

    num_obs = atoi(sTemp);

    ROS_INFO("%i objects in file",num_obs);

    //get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i=0; i < num_obs; ++i) {
        if (fscanf(fCfg,"%s",sTemp) < 1) {
            printf("Parsed string has length < 1.\n");
        }
        object_ids.push_back(sTemp);

        objects[i].resize(6);
        for (int j=0; j < 6; ++j)
        {
            if (fscanf(fCfg,"%s",sTemp) < 1) {
                printf("Parsed string has length < 1.\n");
            }
            if (!feof(fCfg) && strlen(sTemp) != 0) {
                objects[i][j] = atof(sTemp);
            }
        }
    }

    return GetCollisionCubes(objects, object_ids, frame_id);
}

std::vector<moveit_msgs::CollisionObject> GetMultiRoomMapCollisionCubes(
    const std::string& frame_id,
    const double x_max,
    const double y_max,
    const double door_width,
    int n_tables){
    // Wall Thickness
    const double th = 0.05;
    // Wall Height
    const double ht = 1.5;
    // Alley Width
    const double alley_width = 2;

    std::string object_id;
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3,0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    // Description of Map:
    // Two columns of rooms separated by an alley.
    // The first column has three equal size rooms vertically arranged.
    // The bottom room has a door in the center.
    // The top room has a door at far-right.
    //
    // The second column has two equal size rooms vertically arranged.
    // They have a door joining them at the center of the common wall.
    //
    // The alley has a door to access each room except the middle room in the
    // first column. All these doors are at the center of the wall shared with
    // the alley.

    // Map
    double map_half_y = y_max/2;
    double map_half_x = x_max/2;
    double map_third_y = y_max/3;
    double map_third_x = x_max/3;
    double d = door_width/2;
    double z = ht/2;

    // Alley
    double alley_left_wall_x = map_half_x;
    double alley_right_wall_x = alley_left_wall_x + alley_width;

    // Alley bottom left wall
    object_id = "wall1";
    pose.position.x = alley_left_wall_x;
    pose.position.y = ((map_third_y/2) - d)/2;
    pose.position.z = z;
    dims[0] = th;
    dims[1] = map_third_y/2 - d;
    dims[2] = ht;
    objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));

    // Alley bottom right wall
    object_id = "wall2";
    pose.position.x = alley_right_wall_x;
    pose.position.y = (map_half_y/2 - d)/2;
    pose.position.z = z;
    dims[0] = th;
    dims[1] = 2*pose.position.y;
    dims[2] = ht;
    objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));

    // First column, bottom room
    double top_wall_length = map_half_x/2 - d;
    object_id = "wall3";
    pose.position.x = top_wall_length/2;
    pose.position.y = map_third_y;
    pose.position.z = z;
    dims[0] = top_wall_length;
    dims[1] = th;
    dims[2] = ht;
    objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));

    object_id = "wall4";
    pose.position.x = map_half_x - top_wall_length/2;
    pose.position.y = map_third_y;
    pose.position.z = z;
    dims[0] = top_wall_length;
    dims[1] = th;
    dims[2] = ht;
    objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));

    double second_col_bottom_wall_length = map_half_y/2 - d;
    // Alley
    object_id = "wall5";
    pose.position.x = alley_right_wall_x;
    pose.position.y = map_half_y;
    pose.position.z = z;
    dims[0] = th;
    dims[1] = map_half_y - 2*d;
    dims[2] = ht;
    objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));

    object_id = "wall6";
    pose.position.x = alley_left_wall_x;
    pose.position.y = map_half_y;
    pose.position.z = z;
    dims[0] = th;
    dims[1] = 2*map_third_y - 2*d;
    dims[2] = ht;
    objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));

    // Second Column
    double common_wall_length = (map_half_x - alley_width)/2 - d;
    object_id = "wall8";
    pose.position.x = alley_right_wall_x + common_wall_length/2;
    pose.position.y = map_half_y;
    pose.position.z = z;
    dims[0] = common_wall_length;
    dims[1] = th;
    dims[2] = ht;
    objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));

    object_id = "wall9";
    pose.position.x = x_max - common_wall_length/2;
    pose.position.y = map_half_y;
    pose.position.z = z;
    dims[0] = common_wall_length;
    dims[1] = th;
    dims[2] = ht;
    objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));

    //Tables
    if(n_tables > 0){
        object_id = "table1";
        pose.position.x = alley_right_wall_x + common_wall_length;
        pose.position.y = map_half_y/2;
        pose.position.z = 0.30;
        dims[0] = 1;
        dims[1] = 1;
        dims[2] = 0.60;
        objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));
    }

    // First Column
    object_id = "wall10";
    pose.position.x = map_half_x/2;
    pose.position.y = 2*map_third_y;
    pose.position.z = z;
    dims[0] = map_half_x - 2*d;
    dims[1] = th;
    dims[2] = ht;
    objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));

    //Second Column
    object_id ="wall11";
    pose.position.x = alley_right_wall_x;
    pose.position.y = y_max - second_col_bottom_wall_length/2;
    pose.position.z = z;
    dims[0] = th;
    dims[1] = second_col_bottom_wall_length;
    dims[2] = ht;
    objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));

    // First Column
    object_id ="wall12";
    pose.position.x = alley_left_wall_x;
    pose.position.y = y_max - (map_third_y/2 - d)/2;
    pose.position.z = z;
    dims[0] = th;
    dims[1] = map_third_y/2 - d;
    dims[2] = ht;
    objs.push_back(GetCollisionCube(pose, dims, frame_id, object_id));

    return objs;
}
