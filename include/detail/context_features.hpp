#ifndef WALKER_CONTEXT_FEATURES_IMPLEMENTATION_H
#define WALKER_CONTEXT_FEATURES_IMPLEMENTATION_H

#include <algorithm>
#include "../context_features.h"

#define LOG "context_features"

/*template <>
MobManipDiscreteFeatures<2>::MobManipDiscreteFeatures(
        smpl::ManipLattice _manip_lattice,
        smpl::Bfs3DHeuristic* _bfs_3d,
        smpl::Bfs3DBaseHeuristic* _bfs_3d_base,
        smpl::Bfs2DHeuristic* _bfs_2d ) :
    m_env{_manip_lattice}, m_bfs_3d{_bfs_3d}, m_bfs_3d_base{_bfs_3d_base},
    m_bfs_2d{_bfs_2d}
{

}

template <>
MobManipDiscreteFeatures<2>::~MobManipDiscreteFeatures()
{

}

template <>
ContextArray MobManipDiscreteFeatures<2>::getContext( vector<double> _robot_state )
{
    ContextArray context;

    // Get state-id
    smpl::RobotCoord robot_coord(_robot_state.size());
    m_env->stateToCoord( _robot_state, robot_coord );
    int state_id = m_env->getOrCreateState(robot_coord);

    int arm_thresh = (int) ( m_arm_len / m_bfs_3d->grid()->resolution() );

    int end_eff_dist = m_bfs_3d->GetGoalHeuristic(state_id);
    int base_dist = m_bfs_2d->GetGoalHeuristic(state_id);
    context[0] = (int)( base_dist < arm_thresh );
    context[1] = ;
    return context;
}

template <>
void MobManipDiscreteFeatures<2>::setRobotArmLength( double _len )
{
    m_arm_length = _len;
}*/

//

MobManipDiscreteFeatures<4>::MobManipDiscreteFeatures(
        smpl::KDLRobotModel* _rm,
        smpl::ManipLattice* _manip_lattice,
        smpl::Bfs2DHeuristic* _bfs_2d,
        smpl::Bfs3DBaseHeuristic* _bfs_3d_base,
        smpl::Bfs3DHeuristic* _bfs_3d) :
    AbstractContext<4, int>(),
    m_rm{_rm}, m_env{_manip_lattice}, m_bfs_2d{_bfs_2d},
    m_bfs_3d_base{_bfs_3d_base}, m_bfs_3d{_bfs_3d}
{

}

auto MobManipDiscreteFeatures<4>::getContext( int _state_id )
    -> ContextArray
{
    ROS_DEBUG_NAMED(LOG, "getContext for %d", _state_id);
    ContextArray context;
    auto grid = m_bfs_3d->grid();
    double res = grid->resolution();
    int cost_per_cell = m_bfs_3d->costPerCell();

    auto state = m_env->getHashEntry(_state_id);
    // Get state-id
    auto robot_coord = state->coord;
    auto robot_state = state->state;

    context[0] = m_bfs_3d_base->GetGoalHeuristic(_state_id) / cost_per_cell;
    context[1] = m_bfs_3d->GetGoalHeuristic(_state_id) / cost_per_cell;

    // Find circum-radius of the robot.
    auto end_eff_frame = m_rm->computeFK(robot_state);
    auto base_link_frame = m_rm->getLinkTransform("base_link");
    assert(base_link_frame!= nullptr);
    auto base_link_to_end_eff = base_link_frame->inverse() * end_eff_frame;
    auto trans = base_link_to_end_eff.translation();
    int circum_radius = (int) ( sqrt(trans.x()*trans.x() + trans.y()*trans.y()) / res );

    context[2] = circum_radius;

    // Find BFS2D_Base path and compute distance to the nearest narrow passage.
    auto path_to_goal = m_bfs_3d_base->getPathToGoal(robot_state);
    // Ignore narrow passages beyond 1m.
    const int MAX_HORIZON = 20;
    context[3] = 0;
    for(int i = 0; i < std::min(MAX_HORIZON, (int) path_to_goal.size()); i++)
    {
        auto point = path_to_goal[i];
        if( grid->getDistance(point[0], point[1], 15) / res < circum_radius)
        {
            context[3] = 1;
            break;
        }
    }
    ROS_DEBUG_NAMED(LOG, "Context for state: %d", _state_id);
    ROS_DEBUG_NAMED(LOG, "  3D-base  3D   Circum-radius  2D Path length");
    ROS_DEBUG_NAMED(LOG, "  %d,      %d    %d,           %d", context[0], context[1], context[2], context[3]);

    return context;
}

void MobManipDiscreteFeatures<4>::setRobotArmLength( double _len )
{
    m_arm_len = _len;
}

#undef LOG

#endif
