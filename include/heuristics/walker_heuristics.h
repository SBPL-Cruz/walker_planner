#ifndef WALKER_HEURISTICS_H
#define WALKER_HEURISTICS_H

#include <stdlib.h>

#include <ros/ros.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <sbpl/planners/types.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/euclid_diffdrive_heuristic.h>
#include <smpl/heuristic/arm_retract_heuristic.h>
#include <smpl/heuristic/base_rot_bfs_heuristic.h>
#include <smpl/heuristic/mother_heuristic.h>
#include <smpl/graph/manip_lattice_multi_rep.h>
#include "config/planner_config.h"
#include "motion_planner.h"

//#define NUM_QUEUES 33
#define NUM_QUEUES 20
//#define NUM_QUEUES 23 //1 + 3 + 3 + 16
#define NUM_ACTION_SPACES 3

static const int DefaultCostMultiplier = 1000;

struct AnchorHeuristic : public smpl::CompoundBfsHeuristic
{
    int GetGoalHeuristic(int state_id) override {
        return std::max(bfs_3d_base->GetGoalHeuristic(state_id), bfs_3d->GetGoalHeuristic(state_id));
    }

    double getMetricGoalDistance(double x, double y, double z) override {
        return bfs_3d->getMetricGoalDistance(x, y, z);
    }

    double getMetricStartDistance(double x, double y, double z) override {
        return bfs_3d->getMetricStartDistance(x, y, z);
    }
};

struct EndEffHeuristic : public smpl::CompoundBfsHeuristic
{
    bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
            std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d){
        if(!smpl::CompoundBfsHeuristic::init(_bfs_3d_base, _bfs_3d))
            return false;
        pose_ext = bfs_3d->planningSpace()->getExtension<smpl::PoseProjectionExtension>();
        return true;
    }

    int GetGoalHeuristic(int state_id){
        if (state_id == bfs_3d->planningSpace()->getGoalStateID()) {
            return 0;
        }
        if(pose_ext == nullptr)
            return 0;
        smpl::Affine3 p;
        if(!pose_ext->projectToPose(state_id, p))
            return 0;

        auto goal_pose = bfs_3d->planningSpace()->goal().pose;

        smpl::Quaternion qa(p.rotation());
        smpl::Quaternion qb(goal_pose.rotation());
        double dot = qa.dot(qb);
        if (dot < 0.0) {
            qb = smpl::Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
            dot = qa.dot(qb);
        }
        int rot_dist = DefaultCostMultiplier*smpl::angles::normalize_angle(2.0 * std::acos(dot));

        int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
        int arm_dist = bfs_3d->GetGoalHeuristic(state_id);

        int heuristic = base_coeff*base_dist + arm_coeff*arm_dist + rot_coeff*rot_dist;
        return heuristic;
    }

    double base_coeff=0.0;
    double arm_coeff=1.0;
    double rot_coeff=5.0;
    smpl::PoseProjectionExtension* pose_ext = nullptr;
};

struct RetractArmHeuristic : public smpl::CompoundBfsHeuristic
{
    bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
            std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d){
        if(!smpl::CompoundBfsHeuristic::init(_bfs_3d_base, _bfs_3d))
            return false;
        return true;
    }

    int GetGoalHeuristic(int state_id){
        if(state_id == 0)
            return 0;

        smpl::Vector3 p;
        if(!bfs_3d->m_pp->projectToPoint(state_id, p)){
            ROS_ERROR("RetractArmHeuristic Could not project");
            return 0;
        }
        auto retracted_robot_state = bfs_3d->planningSpace()->getExtension<smpl::ExtractRobotStateExtension>()->extractState(state_id);

        // Norm of first 5 joints.
        double norm = 0.0;
        for(int i=3; i<8; i++)
            norm += (retracted_robot_state[i]*retracted_robot_state[i]);
        norm = sqrt(norm);
        int retract_heuristic = DefaultCostMultiplier * norm;

        int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);

        int heuristic = base_coeff*base_dist + retract_arm_coeff*retract_heuristic;

        return heuristic;
    }

    double base_coeff = 0.0;
    double retract_arm_coeff = 10.0;

};

struct ImprovedEndEffHeuristic : public smpl::CompoundBfsHeuristic
{
    bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
            std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d,
            std::shared_ptr<RetractArmHeuristic> _retract_arm){
        if(!smpl::CompoundBfsHeuristic::init(_bfs_3d_base, _bfs_3d))
            return false;
        m_retract_arm_heur = _retract_arm;
        pose_ext = bfs_3d->planningSpace()->getExtension<smpl::PoseProjectionExtension>();
        return true;
    }

    int GetGoalHeuristic(int state_id){
        if (state_id == bfs_3d->planningSpace()->getGoalStateID()) {
            return 0;
        }
        if(pose_ext == nullptr)
            return 0;
        smpl::Affine3 p;
        if(!pose_ext->projectToPose(state_id, p))
            return 0;

        auto goal_pose = bfs_3d->planningSpace()->goal().pose;

        smpl::Quaternion qa(p.rotation());
        smpl::Quaternion qb(goal_pose.rotation());
        double dot = qa.dot(qb);
        if (dot < 0.0) {
            qb = smpl::Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
            dot = qa.dot(qb);
        }
        int rot_dist = DefaultCostMultiplier*smpl::angles::normalize_angle(2.0 * std::acos(dot));
        int base_dist;
        if(bfs_3d_base != nullptr)
             base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
        else
            base_dist = 0;
        auto arm_dist = bfs_3d->GetGoalHeuristic(state_id);
        if(arm_dist/bfs_3d->getCostPerCell() > 30){
            arm_dist = m_retract_arm_heur->GetGoalHeuristic(state_id);
            rot_dist = 0.0;
        }
        int heuristic = base_coeff*base_dist + arm_coeff*arm_dist + rot_coeff*rot_dist;
        return heuristic;
    }

    double base_coeff=0.5;
    double arm_coeff=0.2;
    //double rot_coeff=0.5;
    double rot_coeff=0.2;

    std::shared_ptr<RetractArmHeuristic> m_retract_arm_heur;
    smpl::PoseProjectionExtension* pose_ext = nullptr;
};

struct BaseRotHeuristic : public smpl::CompoundBfsHeuristic
{
    bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
            std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d,
            std::shared_ptr<RetractArmHeuristic> _retract_arm,
            double _orientation){
        if(!smpl::CompoundBfsHeuristic::init(_bfs_3d_base, _bfs_3d))
            return false;
        m_retract_arm_heur = _retract_arm;
        orientation = _orientation;
        return true;
    }

    int GetGoalHeuristic(int state_id){
        if(state_id == 0)
            return 0;
        auto robot_state = (dynamic_cast<smpl::ManipLattice*>(
                    bfs_3d->planningSpace()))->extractState(state_id);
        int yaw_dist = DefaultCostMultiplier*smpl::angles::shortest_angle_dist(robot_state[2], orientation);

        int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
        double arm_fold_heur = 0.0;
        if(base_dist > 10000)
            arm_fold_heur = m_retract_arm_heur->GetGoalHeuristic(state_id);

        int heuristic = base_coeff*base_dist + orientation_coeff*yaw_dist + arm_fold_coeff*arm_fold_heur;
        //ROS_ERROR("%d + %d + %d = %d", int(base_dist), int(orientation_coeff*yaw_dist), int(arm_fold_coeff*arm_fold_heur), heuristic);
        return heuristic;
    }

    std::shared_ptr<RetractArmHeuristic> m_retract_arm_heur;
    double base_coeff = 0.5;
    double orientation = 0.0;
    double orientation_coeff = 0.5;
    double arm_fold_coeff = 0.1;
};

bool constructHeuristics(
        std::array< std::shared_ptr<smpl::RobotHeuristic>, NUM_QUEUES >& heurs,
        std::array<int, NUM_QUEUES>& rep_ids,
        std::vector< std::shared_ptr<smpl::RobotHeuristic> >& bfs_heurs,
        smpl::ManipLattice* pspace,
        smpl::OccupancyGrid* grid,
        smpl::KDLRobotModel* rm,
        PlannerConfig& params );
#endif
