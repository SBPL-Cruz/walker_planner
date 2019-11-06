#ifndef START_GOAL_GENERATOR_H
#define START_GOAL_GENERATOR_H

#include <string>
#include <vector>
#include <utility>

#include <smpl/graph/goal_constraint.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <sbpl_collision_checking/collision_space.h>

// Helper Struct to let the user specify a Start region with respect to some
// reference state.
struct BoundedRegion {
    std::vector<double> lo;
    std::vector<double> hi;
};

// Represents a region as a collections of continuous subregions, each of which
// is defined by the lo and hi bounds of the subregion.
class Region {
    public:
    using subregion = std::pair<std::vector<double>, std::vector<double>>;

    std::vector<double> getRandState();
    std::vector<double> getRandStateInSubregion(int i);
    bool isValid(std::vector<double>);

    std::vector<subregion> subregions;
};

template <typename RobotModel = smpl::KDLRobotModel>
class StartGoalGenerator {
    public:
    bool init( smpl::collision::CollisionSpace*, RobotModel*, unsigned int seed );
    bool addStartRegion(BoundedRegion&);
    bool addGoalRegion(BoundedRegion&);
    //* Generate N start-goal pairs uniformly randomly among the added start
    //and goal regions.
    bool generate(int N);
    bool writeToFile(std::string start_header,
            std::string start_file_name,
            std::string goal_file_name,
            std::string goal_pose_file_name);
    void clear();

    private:
    smpl::collision::CollisionSpace* m_cc = nullptr;
    RobotModel* m_rm = nullptr;

    Region m_start_region;
    Region m_goal_region;

    std::vector<smpl::RobotState> m_start_states;
    std::vector<std::vector<double>> m_goal_poses;
    std::vector<smpl::RobotState> m_goal_states;
};

#include "detail/start_goal_generator.hpp"

#endif
