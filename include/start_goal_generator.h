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
struct StartConstraint {
    std::vector<double> reference;
    std::vector<double> tol;
};

// Represents a region as a collections of continuous subregions, each of which
// is defined by the lo and hi bounds of the subregion.
struct Region {
    using subregion = std::pair<std::vector<double>, std::vector<double>>;

    bool isValid(std::vector<double>);
    std::vector<subregion> subregions;
};

class StartGoalGenerator {
    public:
    bool init( smpl::collision::CollisionSpace*, smpl::urdf::URDFRobotModel*, unsigned int seed );
    bool addStartRegion(StartConstraint&);
    bool addGoalRegion(smpl::GoalConstraint&);
    bool generate(int);
    bool writeToFile(std::string);
    void clear();

    private:
    double getRandNum(double, double);
    smpl::collision::CollisionSpace* m_cc = nullptr;
    smpl::urdf::URDFRobotModel* m_rm = nullptr;

    Region m_start_region;
    Region m_goal_region;
};

#endif
