#ifndef WALKER_MRMHAPLANNER_CONTEXT_FEATURES_H
#define WALKER_MRMHAPLANNER_CONTEXT_FEATURES_H

#include <vector>
#include <array>

#include <smpl/graph/manip_lattice.h>
#include <smpl/heuristic/bfs_2d_heuristic.h>
#include <smpl/heuristic/bfs_3d_heuristic.h>
#include <smpl/heuristic/bfs_3d_base_heuristic.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>

#include "context.h"

template <int N>
class MobManipDiscreteFeatures : public AbstractContext<N, int>
{
    public:
    /**Features:
     * BFS3D distance to goal
     * BFS2D distance to base pose near goal
     * Distance to nearest narrow passage on 2D BFS path
     * Arm-retract state
     **/
    using ContextArray = std::array<int, N>;
    ContextArray getContext( const std::vector<double>& ) override
    {
        throw "Not Implemented";
    }
};

/*template <>
class MobManipDiscreteFeatures<2> : public AbstractContext<2, int>
{
    public:
    MobManipDiscreteFeatures(smpl::ManipLattice*,
        smpl::Bfs3DHeuristic*,
        smpl::Bfs3DBaseHeuristic*,
        smpl::Bfs2DHeuristic* );
    ~MobManipDiscreteFeatures();

    using ContextArray = std::array<2, int>;
    ContextArray getContext( std::vector<double> ) override;

    void setRobotArmLength( double );

    private:
    smpl::ManipLattice m_env;
    smpl::Bfs3DHeuristic* m_bfs_3d;
    smpl::Bfs3DBaseHeuristic* m_bfs_3d_base;
    smpl::Bfs2DHeuristic* m_bfs_2d;

    double m_arm_len = 0.0;
};*/

template <>
class MobManipDiscreteFeatures<4> : public AbstractContext<4, int>
{
    public:
    MobManipDiscreteFeatures(
            smpl::KDLRobotModel* rm,
            smpl::ManipLattice*,
            smpl::Bfs2DHeuristic*,
            smpl::Bfs3DBaseHeuristic*,
            smpl::Bfs3DHeuristic*);

    /**Features:
     * BFS2D distance to goal
     * BFS2D distance to base pose near goal
     * Circum radius
     * Distance to nearest narrow passage on 2D BFS path
     **/
    using ContextArray = std::array<int, 4>;
    ContextArray getContext( const std::vector<double>& ) override;

    void setRobotArmLength( double );

    private:
    smpl::KDLRobotModel* m_rm = nullptr;
    smpl::ManipLattice* m_env = nullptr;
    smpl::Bfs2DHeuristic* m_bfs_2d = nullptr;
    smpl::Bfs3DBaseHeuristic* m_bfs_3d_base = nullptr;
    smpl::Bfs3DHeuristic* m_bfs_3d = nullptr;

    double m_arm_len = 0.0;
};

#include "detail/context_features.hpp"

#endif
