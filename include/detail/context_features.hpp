#ifndef WALKER_CONTEXT_FEATURES_IMPLEMENTATION_H
#define WALKER_CONTEXT_FEATURES_IMPLEMENTATION_H

#include <algorithm>
#include <leatherman/bresenham.h>
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
    auto robot_state = state->state;
    std::vector<int> base_coord;
    for(int i = 0;i < 3; i++)
    {
        int coord;
        if(robot_state[i] >= 0)
        {
            coord = (int)(robot_state[i] / m_env->resolutions()[i] + 0.5);
        } else {
            coord = (int)(robot_state[i] / m_env->resolutions()[i] - 0.5);
        }
        base_coord.push_back(coord);
    }

    // 0 = Base distance to goal
    auto goal_xyz = m_env->goal().pose.translation();
    auto goal_rot = m_env->goal().pose.rotation();

    int dgoal_xyz[3];
    grid->worldToGrid(goal_xyz[0], goal_xyz[1], goal_xyz[2], dgoal_xyz[0], dgoal_xyz[1], dgoal_xyz[2]);

    context[0] = sqrt( ( dgoal_xyz[0] - base_coord[0] )*(dgoal_xyz[0] - base_coord[0]) +
            (dgoal_xyz[1] - base_coord[1])*(dgoal_xyz[1] - base_coord[1]));

    // For obstacles
    // Optional
    //context[0] = m_bfs_3d_base->GetGoalHeuristic(_state_id) / cost_per_cell;

    // For obstacles
    context[1] = m_bfs_3d->GetGoalHeuristic(_state_id) / cost_per_cell;

    // Dist between base yaw and goal yaw
    double y, p, r;
    smpl::angles::get_euler_zyx(goal_rot, y, p, r);
    double theta_res = m_env->resolutions()[2];
    //context[2] = smpl::angles::shortest_angle_dist(robot_state[2], y) / theta_res;

    // Find circum-radius of the robot.
    auto end_eff_frame = m_rm->computeFK(robot_state);
    auto base_link_frame = m_rm->getLinkTransform("base_link");
    assert(base_link_frame!= nullptr);
    auto base_link_to_end_eff = base_link_frame->inverse() * end_eff_frame;
    auto trans = base_link_to_end_eff.translation();
    int circum_radius = (int) ( sqrt(trans.x()*trans.x() + trans.y()*trans.y()) / res + 0.5 );
    // We check for obstacles at a height of 15*res in the occupancy grid.
    circum_radius = std::min(circum_radius, 14);

    context[2] = circum_radius;

    // Find BFS2D_Base path and compute distance to the nearest narrow passage.
    auto path_to_goal = m_bfs_3d_base->getPathToGoal(robot_state);
    // Ignore narrow passages beyond 1.5m.
    const int MAX_HORIZON = 30;
    context[3] = 0;
    for(int i = 0; i < std::min(MAX_HORIZON, (int) path_to_goal.size()); i++)
    {
        auto& point = path_to_goal[i];
        context[3] = i;
        //ROS_WARN( "Dist to narrow-passage: %f, circum-radius: %f", (grid->getDistance(point[0], point[1], 15)), res*(double)circum_radius );
        if( grid->getDistance(point[0], point[1], 15) < (res*circum_radius) )
        {
            break;
        }
    }

    // Ray tracing.
    // From end-effector to Goal.
    //int dend_eff_xyz[3];
    //grid->worldToGrid(
            //end_eff_frame.translation()[0], end_eff_frame.translation()[1], end_eff_frame.translation()[2],
            //dend_eff_xyz[0], dend_eff_xyz[1], dend_eff_xyz[2]);

    //std::vector<int> ray_to_goal;
    //std::vector<int> ray_to_goal(1000);
    //auto ray_end_it = ray_to_goal.begin();
    //leatherman::GetBresenhamLine(dend_eff_xyz, dgoal_xyz, ray_end_it);

    //// All clear
    //context[3] = 1;
    //// Checking only within 1 m from goal.
    //int ray_len = ray_end_it - ray_to_goal.begin();
    //if(ray_len <= 60)
    //{
        //int point[3];
        //for(int i = 0; i < ray_len; i+=3)
        //{
            //point[0] = ray_to_goal[i%3];
            //point[1] = ray_to_goal[i%3 + 1];
            //point[2] = ray_to_goal[i%3 + 2];
            //// Cluttered ray
            //if(grid->getDistance(point[0], point[1], point[2]) < 0.1)
            //{
                //context[3] = 0;
                //break;
            //}
        //}
    //}

    // Base dist
    context[0] = std::min(context[0], 16);
    // 3D BFS
    context[1] = std::min(context[1], 16);
    //Circumradius
    context[2] = std::min(context[2], 14);
    // Dist to narrow passage
    context[3] = std::min(context[3], 10);

    ROS_DEBUG_NAMED(LOG, "Context for state: %d", _state_id);
    //ROS_DEBUG_NAMED(LOG, "  3D-base  3D   Circum-radius  Dist-to-narrow-passage");
    //ROS_DEBUG_NAMED(LOG, "    Base Dist to goal  BFS-End-Eff      Yaw dist to goal    Dist-to-narrow-passage\n");
    ROS_DEBUG_NAMED(LOG, "  %d,                   %d,              %d,                %d", context[0], context[1], context[2], context[3]);

    // Dist from base
    if(context[0] > 10)
        context[0] = 1;
    else
        context[0] = 0;

    if(context[1] > 2)
        context[1] = 1;
    else
        context[1] = 0;

    if(context[2] > 6)
        context[2] = 1;
    else
        context[2] = 0;

    if(context[3] > 3)
        context[3] = 1;
    else
        context[3] = 0;

    //ros::Duration(0.1).sleep();
    return context;
}

void MobManipDiscreteFeatures<4>::setRobotArmLength( double _len )
{
    m_arm_len = _len;
}

#undef LOG

#endif
