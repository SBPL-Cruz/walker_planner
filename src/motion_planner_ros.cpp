#include "motion_planner_ros.h"

bool IsMultiDOFJointVariable(
    const std::string& name,
    std::string* joint_name,
    std::string* local_name)
{
    auto slash_pos = name.find_last_of('/');
    if (slash_pos != std::string::npos) {
        if (joint_name != NULL) {
            *joint_name = name.substr(0, slash_pos);
        }
        if (local_name != NULL) {
            *local_name = name.substr(slash_pos + 1);
        }
        return true;
    } else {
        return false;
    }
}


std::vector<double> getResolutions(
        smpl::RobotModel* robot,
        const PlannerConfig& params ){

    auto resolutions = std::vector<double>(robot->jointVariableCount());

    std::string disc_string = params.discretization;
    if (disc_string.empty()) {
        ROS_ERROR("Parameter 'discretization' not found in planning params");
    }

    auto disc = ParseMapFromString<double>(disc_string);

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        auto& vname = robot->getPlanningJoints()[vidx];
        std::string joint_name, local_name;
        if (IsMultiDOFJointVariable(vname, &joint_name, &local_name)) {
            // adjust variable name if a variable of a multi-dof joint
            auto mdof_vname = joint_name + "_" + local_name;
            auto dit = disc.find(mdof_vname);
            if (dit == end(disc)) {
                ROS_ERROR("Discretization for variable '%s' not found in planning parameters", vname.c_str());
            }
            resolutions[vidx] = dit->second;
        } else {
            auto dit = disc.find(vname);
            if (dit == end(disc)) {
                ROS_ERROR("Discretization for variable '%s' not found in planning parameters", vname.c_str());
            }
            resolutions[vidx] = dit->second;
        }

        ROS_ERROR("resolution(%s) = %0.3f", vname.c_str(), resolutions[vidx]);
    }

    return resolutions;
}
/*
void Callbacks::publish_path(
        std::vector<smpl::RobotState> path,
        ros::Publisher path_pub ){
    ROS_ERROR("publishing path");
    int number_of_joints = 16;
    ros::Rate r(10);

    for(int k=0; k<10000;k++){
    walker_planner::Path1 Final_path;
    for(int i=0;i<path.size();i++){
        walker_planner::GraspPose State;
        for(int j=0;j<path[i].size();j++)
        {
            State.a.push_back(path[i][j]);
        }
        Final_path.path.push_back(State);
    }
    path_pub.publish(Final_path);
    }
}
*/
