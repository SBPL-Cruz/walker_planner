#ifndef CVAE_SCHEDULING_POLICIES_IMPLEMENTATION_H
#define CVAE_SCHEDULING_POLICIES_IMPLEMENTATION_H

#include <ros/ros.h>
#include <numeric>
#include <smpl/debug/marker_utils.h>
#include "../cvae_scheduling_policies.h"

#define FINE_LOG "cvae_scheduling_policy_detail"
#define LOG "cvae_scheduling_policy"

CVAEPolicy::CVAEPolicy( int _num_arms ) :
    SchedulingPolicy(_num_arms)
{}

CVAEPolicy::~CVAEPolicy()
{}

template <typename C>
CVAENNPolicy<C>::CVAENNPolicy( int _num_arms, int _num_reps ) :
    CVAEPolicy(_num_arms)
{
    m_rtrees.resize(_num_reps);
}

template <typename C>
CVAENNPolicy<C>::~CVAENNPolicy()
{}

template <typename C>
int CVAENNPolicy<C>::getAction()
{}

template <typename C>
int CVAENNPolicy<C>::getAction(int _state_id)
{
    throw "Not Implemented.";
}

template <typename C>
int CVAENNPolicy<C>::getArm( const std::vector<C>& _contexts, const std::vector<int>& _rep_ids )
{
    //std::vector<double> probs(_rep_ids.size());
    std::vector<int> likelihoods(_rep_ids.size());

    ROS_DEBUG_NAMED(FINE_LOG, "Rep Probabilities: ");
    for(int i = 0; i < _contexts.size(); i++)
    {
        std::vector< std::vector<Value> > nearest_neighbours(m_rtrees.size());
        // Context is a vector with : x, y of base
        auto& context = _contexts[i];
        int id = _rep_ids[i];
        double radius = 0.2;
        std::vector<int> counts(m_rtrees.size(), 0);
        Box box( Point(context[0] - radius, context[1] - radius),
                Point(context[0] + radius, context[1] + radius) );

        //Ignoring Fullbody representation
        for(int j = 1; j < m_rtrees.size(); j++)
        {
            m_rtrees[j].query( bg::index::within(box),
                    std::back_inserter(nearest_neighbours[j]) );
            counts[j] = nearest_neighbours[j].size();
        }
        if( std::any_of(counts.begin() + 1, counts.end(), [](int x){ return x == 0; }) )
            likelihoods[i] = 100 * 0.5;//-1;
        else
            likelihoods[i] = (int) (100 * ( (double)counts[id] / (double) std::accumulate(counts.begin(), counts.end(), 0) ));
        ROS_DEBUG_NAMED( FINE_LOG, "  Rep: %d, Queue: %d, Likelihood: %d", id, i, likelihoods[i] );
    }

    int arm_id = -1;
    // Chose the best Representation based on representation likelihood
    // computed using nearest neighbour counts.
    std::discrete_distribution<int> distribution(likelihoods.begin(), likelihoods.end());
    arm_id = distribution(m_generator);

    ROS_DEBUG_NAMED(LOG, "Chosen Rep: %d, Arm: %d", _rep_ids[arm_id], arm_id);
    //ros::Duration(0.5).sleep();
    return arm_id;
}

template <typename C>
void CVAENNPolicy<C>::updatePolicy( C& _context, double _reward, int _rep_id )
{

}

template <typename C>
bool CVAENNPolicy<C>::loadRepDistribution(std::string _file_name, int _rep_id)
{
    std::ifstream ifs;
    ifs.open(_file_name, std::ios::in);
    if(!ifs.is_open())
    {
        ROS_ERROR("Could not open Distribution file.");
        return false;
    }
    ROS_DEBUG_NAMED(LOG, "Loading Rep %d", _rep_id);

    std::string line;
    unsigned int idx = 0;
    while(ifs >> line)
    {
        ROS_DEBUG_NAMED(FINE_LOG, "%s", line.c_str());
        double x, y;
        char comma;
        std::istringstream ss(line);
        ss >> x >> comma >> y;
        ROS_DEBUG_NAMED(FINE_LOG, "%f, %f", x, y);
        Point point = Point(x, y);
        m_rtrees[_rep_id].insert(point);
        //m_rtrees[_rep_id].insert(std::make_pair(point, idx));
        SV_SHOW_INFO(smpl::visual::MakeSphereMarker( x, y, 0.0, 0.05, 100.0/_rep_id, "dummy_base", "cvae_base_" + std::to_string(_rep_id), idx ));
        idx++;
    }
    ROS_DEBUG_NAMED( LOG, "    Loaded %d points", m_rtrees[_rep_id].size() );

    ifs.close();
    return true;
}

#endif
