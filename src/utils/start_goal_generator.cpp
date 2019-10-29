#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen_conversions/eigen_msg.h>

#include "utils/start_goal_generator.h"
#include "utils/utils.h"

#define PI 3.14

bool Region::isValid(std::vector<double> _state){
    assert(subregions.size());
    assert(_state.size() == subregions[0].first.size());

    bool validity = false;
    for(auto& subregion : subregions){
        bool inBound = true;
        auto& lo = subregion.first;
        auto& hi = subregion.second;
        for(int i=0; i<_state.size(); i++){
            if(_state[i] < lo[i] || _state[i] > hi[i]){
                inBound = false;
                break;
            }
        }
        if(inBound){
            validity = true;
            break;
        }
    }

    return validity;
}

std::vector<double> Region::getRandState(){
    const int N = subregions[0].first.size();
    std::vector<double> state(N, 0);
    int subregionIx = rand() % subregions.size();
    auto& lo = subregions[subregionIx].first;
    auto& hi = subregions[subregionIx].second;
    for(int i=0; i<state.size(); i++){
        if(fabs(hi[i] - lo[i]) < 0.001)
            state[i] = lo[i];
        else
            state[i] = getRandNum(lo[i], hi[i]);
    }

    return state;
}

bool StartGoalGenerator::init( smpl::collision::CollisionSpace* _cc,
        smpl::KDLRobotModel* _rm,
        unsigned int _seed ){
    m_cc = _cc;
    m_rm = _rm;
    srand(_seed);
    return true;
}

bool StartGoalGenerator::addStartRegion(BoundedRegion& _start){
    m_start_region.subregions.push_back(std::make_pair(_start.lo, _start.hi));
    return true;
}

bool StartGoalGenerator::addGoalRegion(BoundedRegion& _goal){
    m_goal_region.subregions.push_back(std::make_pair(_goal.lo, _goal.hi));
    return true;
}

bool StartGoalGenerator::generate(int _n){
    assert(m_start_region.subregions.size());
    assert(m_goal_region.subregions.size());
    int num_generated = 0;
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
        found = false;
        while(!found){
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

            //int gx, gy, gz;
            //m_cc->grid()->worldToGrid(rand_goal[0], rand_goal[1], rand_goal[2],
            //        gx, gy, gz);
            //if(m_cc->grid()->getDistance(gx, gy, gz) > 0){
            //    m_goal_poses.push_back(rand_goal);
            //    found = true;
            //}
            smpl::RobotState seed_state(m_start_region.getRandState().size(), 0);

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
            //m_heuristic_base_poses.clear();
            for (int i=0; i<500; i++) {
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
                    //m_heuristic_base_poses.push_back(possible_state);
                    seed_state = possible_state;
                    found_base = true;
                    ROS_INFO("Success on Iteration: %d", i);
                    ROS_INFO("Optimal yaw: %f, Found yaw: %f", theta_opt, theta);
                    break;
                }
                // Explore symmetrically about the optimal theta.
                if(i%2)
                    theta = theta_opt + delta;
                else
                    theta = theta_opt - delta;
            }
            if(found_base)
                break;
            }
            if(!found_base){
            double arm_length = 0.45;
            for(int j=3 ;j<10; j++){
                arm_length += arm_increment;
            for (int i=0; i<1500; i++) {
                delta += increment;
                double possible_x = goal_x + arm_length*cos(theta);
                double possible_y = goal_y + arm_length*sin(theta);
                double possible_yaw = atan2(goal_y - possible_y, goal_x - possible_x);
                smpl::RobotState possible_state(seed_state.size(), 0);
                possible_state[0] = possible_x;
                possible_state[1] = possible_y;
                possible_state[2] = possible_yaw;

                if (m_cc->isStateValid(possible_state)) {
                    //m_heuristic_base_poses.push_back(possible_state);
                    seed_state = possible_state;
                    found_base = true;
                    ROS_INFO("Success on Iteration: %d", i);
                    ROS_INFO("Optimal yaw: %f, Found yaw: %f", theta_opt, theta);
                    break;
                }
                // Explore symmetrically about the optimal theta.
                if(i%2)
                    theta = theta_opt + delta;
                else
                    theta = theta_opt - delta;
            }
            if(found_base)
                break;
            }
            }

            seed_state[4] = -1;
            seed_state[5] = -1;

            smpl::RobotState solution;
            auto success = m_rm->computeIK(goal_pose, seed_state, solution);
            if(success){
                ROS_ERROR("Solved IK");
                if(m_cc->isStateValid(solution)){
                    m_goal_states.push_back(solution);
                    found = true;
                }
                else{
                    ROS_ERROR("Goal IK in collision");
                }
            } else{
                ROS_WARN("Could not solve IK");
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

bool StartGoalGenerator::writeToFile(std::string _start_header, std::string _start_file_name, std::string _goal_file_name){
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
        //for(auto& pose : m_goal_poses){
        //    for(auto& val : pose){
        //        goal_stream << val <<" ";
        //    }
        //    goal_stream << "\n";
        //}
        //goal_stream.close();
        for(auto& state : m_goal_states){
            for(auto& val : state){
                goal_stream << val <<" ";
            }
            goal_stream << "\n";
        }
        goal_stream.close();
    }

    return true;
}

void StartGoalGenerator::clear(){
    m_start_region.subregions.clear();
    m_goal_region.subregions.clear();
    m_start_states.clear();
    //m_goal_poses.clear();
    m_goal_states.clear();
}
