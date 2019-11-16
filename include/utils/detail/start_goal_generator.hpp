#ifndef START_GOAL_GENERATOR_IMPLEMENTATION_H
#define START_GOAL_GENERATOR_IMPLEMENTATION_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen_conversions/eigen_msg.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>

#include "config/planner_config.h"
#include "utils/utils.h"
#include "../start_goal_generator.h"

#define PI M_PI
#define MAX_ITERS 100

template <typename RM>
bool StartGoalGenerator<RM>::init( smpl::collision::CollisionSpace* _cc,
        RM* _rm,
        unsigned int _seed ){
    m_cc = _cc;
    m_rm = _rm;
    srand(_seed);
    return true;
}

template <typename RM>
bool StartGoalGenerator<RM>::addStartRegion(BoundedRegion& _start){
    m_start_region.subregions.push_back(std::make_pair(_start.lo, _start.hi));
    return true;
}

template <typename RM>
bool StartGoalGenerator<RM>::addGoalRegion(BoundedRegion& _goal){
    m_goal_region.subregions.push_back(std::make_pair(_goal.lo, _goal.hi));
    return true;
}

template <typename RM>
bool StartGoalGenerator<RM>::generate(int _n){
    assert(m_start_region.subregions.size());
    assert(m_goal_region.subregions.size());
    int num_generated = 0;
    int fullbody_ik_successes = 0;
    int iters = 0;
    const int max_iters = 1000;
    while(num_generated < _n && iters < max_iters){
        iters++;
        //Start
        bool found = false;
        while(!found){
            auto rand_start = m_start_region.getRandState();

            if(m_cc->isStateValid(rand_start)){
                m_start_states.push_back(rand_start);
                found = true;
            } else
                ROS_ERROR("Start in collision");
        }

        //Goal
        smpl::RobotState seed_state(this->m_start_region.getRandState().size(), 0);
        // Will find a robot state around this end-eff pose.
        auto rand_goal = m_goal_region.getRandState();

        smpl::Affine3 goal_pose;
        geometry_msgs::Pose read_grasp;
        read_grasp.position.x = rand_goal[0];
        read_grasp.position.y = rand_goal[1];
        read_grasp.position.z = rand_goal[2];
        tf2::Quaternion q;
        q.setRPY( rand_goal[3], rand_goal[4], rand_goal[5] );
        read_grasp.orientation.x = q[0];
        read_grasp.orientation.y = q[1];
        read_grasp.orientation.z = q[2];
        read_grasp.orientation.w = q[3];
        tf::poseMsgToEigen( read_grasp, goal_pose);

        SV_SHOW_INFO_NAMED("goal_pose", smpl::visual::MakePoseMarkers(goal_pose, "dummy_base", "goal_pose"));

        //Get a decent base position for this end-eff pose.
        double goal_x = goal_pose.translation()[0];
        double goal_y = goal_pose.translation()[1];
        auto rot = goal_pose.rotation();

        double r, p, ya;
        smpl::angles::get_euler_zyx(rot, ya, p, r);

        double base_x=0, base_y=0;

        double delta = 0;
        double increment = 0.005;
        double arm_increment = 0.05;

        bool found_base = false;

        double theta_opt = PI/2 + ya;
        double theta = theta_opt;
        bool found_goal_state = false;
        for (int i=0; i<500; i++){
            delta += increment;
            double arm_length = 0.45;
            for(int j=0 ;j<3; j++){
                arm_length += arm_increment;
                double possible_x = goal_x + arm_length*cos(theta);
                double possible_y = goal_y + arm_length*sin(theta);
                double possible_yaw = atan2(goal_y - possible_y, goal_x - possible_x);
                smpl::RobotState possible_state(seed_state.size(), 0);
                possible_state[0] = possible_x;
                possible_state[1] = possible_y;
                possible_state[2] = possible_yaw;

                if (m_cc->isStateValid(possible_state)) {
                    int iters = 0;
                    while(!found_goal_state & iters < MAX_ITERS){
                        found_base = true;
                        //ROS_INFO("Success on Iteration: %d", i);
                        //ROS_INFO("Optimal yaw: %f, Found yaw: %f", theta_opt, theta);

                        auto rand_start_state = m_start_region.getRandState();
                        for( int j = 3; j < possible_state.size(); j++)
                            possible_state[j] = rand_start_state[j];

                        auto markers = m_cc->getCollisionRobotVisualization(possible_state);
                        for(auto& marker : markers.markers)
                            marker.ns = "temp_state";
                        SV_SHOW_INFO(markers);
                        //ros::Duration(1.0).sleep();

                        auto end_eff_pose = m_rm->computeFK(possible_state);
                        auto pose_xyzrpy = poseToXYZRPY(end_eff_pose);
                        if(m_goal_region.isValid(pose_xyzrpy) && m_cc->isStateValid(possible_state)){
                            ROS_ERROR("Found a valid goal robot state");
                            //ros::Duration(1.0).sleep();
                            m_goal_states.push_back(possible_state);
                            m_goal_poses.push_back(pose_xyzrpy);
                            found_goal_state = true;
                        }
                        iters++;
                    }
                }
                // Explore symmetrically about the optimal theta.
                if(i%2)
                    theta = theta_opt + delta;
                else
                    theta = theta_opt - delta;
            }
            if(found_goal_state){
                break;
            }
        }
        num_generated++;
    }
    //ROS_WARN("Num generated: %d", num_generated);
    if(num_generated != _n)
        return false;
    else
        return true;
}

template <typename RM>
bool StartGoalGenerator<RM>::writeToFile(std::string _start_header,
        std::string _start_file_name,
        std::string _goal_file_name,
        std::string _goal_pose_file_name){
    {
        std::ofstream start_stream;
        start_stream.open(_start_file_name);
        if(!start_stream.is_open()){
            ROS_ERROR("Could not open start file.");
            return false;
        }
        start_stream << _start_header;
        for(auto& state : m_start_states){
            for(auto& val : state){
                start_stream << val <<" ";
            }
            start_stream <<"\n";
        }
        start_stream.close();
    }
    {
        std::ofstream goal_stream;
        goal_stream.open(_goal_file_name);
        if(!goal_stream.is_open()){
            ROS_ERROR("Could not open goal file.");
            return false;
        }
        for(auto& state : m_goal_states){
            for(auto& val : state){
                goal_stream << val <<" ";
            }
            goal_stream << "\n";
        }
        goal_stream.close();
    }
    {
        std::ofstream goal_pose_stream;
        goal_pose_stream.open(_goal_pose_file_name);
        if(!goal_pose_stream.is_open()){
            ROS_ERROR("Could not open goal file.");
            return false;
        }
        for(auto& pose : m_goal_poses){
            for(auto& val : pose){
                goal_pose_stream << val <<" ";
            }
            goal_pose_stream << "\n";
        }
        goal_pose_stream.close();
    }
    return true;
}

template <typename RM>
void StartGoalGenerator<RM>::clear(){
    m_start_region.subregions.clear();
    m_goal_region.subregions.clear();
    m_start_states.clear();
    //m_goal_poses.clear();
    m_goal_states.clear();
}

#endif
