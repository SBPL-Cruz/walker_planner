#ifndef CVAE_SCHEDULING_POLICIES_H
#define CVAE_SCHEDULING_POLICIES_H

#include <vector>
#include <sbpl/planners/scheduling_policy.h>

/** Base Class for Policies making use of CVAE**/
class CVAEPolicy : public SchedulingPolicy
{
    public:
    CVAEPolicy(int num_arms);
    ~CVAEPolicy();

    double getActionSpaceProb( int, int ) override
    {
        throw "Not Implemented";
    }

    virtual int getAction(int state_id) = 0;
};


class CVAENNPolicy : public CVAEPolicy
{
    public:
    CVAENNPolicy(int num_arms);
    ~CVAENNPolicy();

    // Required from CVAEPolicy //
    int getAction(int state_id) override;

    // Reads (x, y) points in a map along with their rep label.
    // Constructs n k-d trees with those points- one for each rep.
    // Constructing different k-d trees is makes query faster (O(log n)).
    int loadRepDistribution(std::string file_name);

    private:
};

#endif
