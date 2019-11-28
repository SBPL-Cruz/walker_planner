#include "cvae_scheduling_policy.h"

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
