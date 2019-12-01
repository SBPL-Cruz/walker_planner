#ifndef CVAE_SCHEDULING_POLICIES_IMPLEMENTATION_H
#define CVAE_SCHEDULING_POLICIES_IMPLEMENTATION_H

#include <ros/ros.h>
#include <numeric>
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
    std::vector< std::vector<Value> > nearest_neighbours(m_rtrees.size());
    for(int i = 0; i < _contexts.size(); i++)
    {
        // Context is a vector with : x, y of base
        auto& context = _contexts[i];
        int id = _rep_ids[i];
        double radius = 5.1;
        Box box( Point(context[0] - radius, context[1] - radius),
                Point(context[0] + radius, context[1] + radius) );
        m_rtrees[id].query( bg::index::within(box),
                std::back_inserter(nearest_neighbours[id]) );
    }
    std::vector<int> counts(m_rtrees.size());
    std::vector<double> probs(m_rtrees.size());
    ROS_DEBUG_NAMED(FINE_LOG, "Rep Counts: ");
    for(auto id : _rep_ids)
    {
        counts[id] = nearest_neighbours[id].size();
        ROS_DEBUG_NAMED(FINE_LOG, "   ID: %d, Count: %d", id, counts[id]);
    }
    int total_count = std::accumulate(counts.begin(), counts.end(), 0);
    int best_id = 0;
    ROS_DEBUG_NAMED(FINE_LOG, "Rep Probabilities: ");
    for(auto id : _rep_ids)
    {
        probs[id] = counts[id] / total_count;
        ROS_DEBUG_NAMED(FINE_LOG, "  ID: %d, Count: %d, Prob: %f", id, counts[id], probs[id]);
        if(probs[id] > probs[best_id])
        {
            best_id = id;
        }
    }
    // XXX Right now it assumes reps:queues::1:1
    return best_id;
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
        idx++;
    }
    ROS_DEBUG_NAMED( LOG, "    Loaded %d points", m_rtrees[_rep_id].size() );

    ifs.close();
    return true;
}

#endif
