#ifndef CVAE_SCHEDULING_POLICIES_H
#define CVAE_SCHEDULING_POLICIES_H

#include <vector>
#include <utility>
#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <sbpl/planners/scheduling_policy.h>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

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
    bool loadRepDistribution(std::string file_name);

    private:
    using Point = bg::model::point<double, 2, bg::cs::cartesian>;
    using Value = std::pair<Point, unsigned int>;
    // Stores base (x, y) points where are representation should be searched.
    // Linear has fastest construction time but slowest query time.
    // rstar has slowest construction time but fastest query time.
    bgi::rtree< Value, bgi::rstar<16> > m_kd_tree_arm;
};

#endif
