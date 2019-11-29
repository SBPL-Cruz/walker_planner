#include <ros/ros.h>
#include "cvae_scheduling_policies.h"

CVAEPolicy::CVAEPolicy( int _num_arms ) :
    SchedulingPolicy(_num_arms)
{}

CVAEPolicy::~CVAEPolicy()
{}

CVAENNPolicy::CVAENNPolicy( int _num_arms ) :
    CVAEPolicy(_num_arms)
{}

CVAENNPolicy::~CVAENNPolicy()
{}

int CVAENNPolicy::getAction(int _state_id)
{
    int action = 0;
    return action;
}

bool CVAENNPolicy::loadRepDistribution(std::string _file_name)
{
    std::ifstream ifs;
    ifs.open(_file_name, std::ios::in);
    if(!ifs.is_open())
    {
        ROS_ERROR("Could not open Distribution file.");
        return false;
    }

    std::string line;
    unsigned int idx = 0;
    while(ifs >> line)
    {
        double x, y;
        char comma;
        std::istringstream ss(line);
        ss >> x >> comma >> y;
        Point point = Point(x, y);
        m_kd_tree_arm.insert(std::make_pair(point, idx));
        idx++;
    }

    ifs.close();
    return true;
}
