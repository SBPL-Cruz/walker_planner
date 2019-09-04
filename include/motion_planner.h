#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <ros/ros.h>

#include <smpl/types.h>
#include <smpl/angles.h>
#include <smpl/time.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/console/console.h>
#include <sbpl/planners/types.h>

namespace MPlanner {

    struct PlannerSolution {
        std::vector<smpl::RobotState> robot_states;
        int cost;
        double planning_time;
        int num_expansions;

        PlannerSolution(){
            robot_states.resize(0);
            cost = 0;
            planning_time = 0;
            num_expansions = 0;
        }

        PlannerSolution(const PlannerSolution& soltn){
            robot_states = soltn.robot_states;
            cost = soltn.cost;
            planning_time = soltn.planning_time;
            num_expansions = soltn.num_expansions;
        }
    };

    struct PlannerParams{
        int planning_time;
        double eps;
        double eps_mha;
        bool search_mode;
    };

}

namespace MPlanner {

    template <typename Search, typename Env>
    class MotionPlanner {
        public:
        MotionPlanner();
        //~MotionPlanner();

        virtual bool init(Search* _search,
                Env* _env,
                Heuristic* _anchor,
                std::vector<Heuristic*>& m_inad,
                PlannerParams&);

        virtual bool updateStart(const smpl::RobotState&);
        virtual bool updateGoal(const smpl::GoalConstraint&);
        virtual bool updatePlannerParams(const PlannerParams&);
        virtual bool plan(PlannerSolution& planner_soltn);

        smpl::RobotModel* robot(){ return m_env_ptr->robot(); }

        private:
        Search* m_search_ptr = nullptr;
        Env* m_env_ptr = nullptr;
        Heuristic* m_anchor_heur_ptr = nullptr;
        std::vector<Heuristic*> m_inad_heurs;

        smpl::RobotState m_start_state;
        smpl::GoalConstraint m_goal_constraints;
        double m_planning_time;
    };

} //namespace MPlanner

namespace MPlanner {

    template <typename Search, typename Env>
    MotionPlanner<Search, Env>::
    MotionPlanner(){
        SMPL_INFO("MotionPlanner Constructor");
    }

    template <typename Search, typename Env>
    bool MotionPlanner<Search, Env>::
    init( Search* _search,
            Env* _env_ptr,
            Heuristic* _anchor_ptr,
            std::vector<Heuristic*>& _inad_heurs,
            PlannerParams& _params ){
        SMPL_INFO("MotionPlanner::init");
        m_search_ptr = _search;
        m_env_ptr = _env_ptr;
        m_anchor_heur_ptr = _anchor_ptr;
        for(auto h : _inad_heurs)
            m_inad_heurs.push_back(h);

        updatePlannerParams(_params);
    }

    template <typename Search, typename Env>
    bool MotionPlanner<Search, Env>::
    updateStart(const smpl::RobotState& _start_state){
        m_start_state = _start_state;
        SMPL_INFO("updateStart");

        if (!m_env_ptr->setStart(_start_state)) {
            ROS_ERROR("Failed to set start state");
            return false;
        }

        auto start_id = m_env_ptr->getStartStateID();
        if (start_id == -1) {
            ROS_ERROR("No start state has been set");
            return false;
        }

        dynamic_cast<smpl::RobotHeuristic*>(m_anchor_heur_ptr)->updateStart(_start_state);
        for (auto& h : m_inad_heurs) {
            dynamic_cast<smpl::RobotHeuristic*>(h)->updateStart(_start_state);
        }

        if (m_search_ptr->set_start(start_id) == 0) {
            ROS_ERROR("Failed to set start state");
            return false;
        }

    }

    template <typename Search, typename Env>
    bool MotionPlanner<Search, Env>::
    updateGoal(const smpl::GoalConstraint& _goal_constraint){
        m_goal_constraints = _goal_constraint;
        SMPL_INFO("updateGoal");

        double yaw, pitch, roll;
        smpl::get_euler_zyx(_goal_constraint.pose.rotation(), yaw, pitch, roll);

        // set sbpl environment goal
        ROS_INFO("Setting goal in pspace");
        if (!m_env_ptr->setGoal(_goal_constraint)) {
            ROS_ERROR("Failed to set goal");
            return false;
        }

        ROS_INFO("Updating goal in heuristics.");
        ROS_INFO("Goal: %f, %f", _goal_constraint.pose.translation()[0], _goal_constraint.pose.translation()[1]);
        assert(m_anchor_heur_ptr != nullptr);
        dynamic_cast<smpl::RobotHeuristic*>(m_anchor_heur_ptr)->updateGoal(_goal_constraint);
        for (auto& h : m_inad_heurs) {
            dynamic_cast<smpl::RobotHeuristic*>(h)->updateGoal(_goal_constraint);
        }

        // set planner goal
        auto goal_id = m_env_ptr->getGoalStateID();
        if (goal_id == -1) {
            ROS_ERROR("No goal state has been set");
            return false;
        }

        ROS_INFO("Setting planner Goal");
        if (m_search_ptr->set_goal(goal_id) == 0) {
            ROS_ERROR("Failed to set planner goal state");
            return false;
        }

        return true;
    }

    template <typename Search, typename Env>
    bool MotionPlanner<Search, Env>::
    updatePlannerParams(const PlannerParams& m_params){
        m_search_ptr->set_initialsolution_eps(m_params.eps);
        m_search_ptr->set_initial_mha_eps(m_params.eps_mha);
        m_search_ptr->set_search_mode(m_params.search_mode);
        m_planning_time = m_params.planning_time;

        return true;
    }

    template <typename Search, typename Env>
    bool MotionPlanner<Search, Env>::
    plan(PlannerSolution& _planner_soltn){
        SMPL_INFO("plan");
        assert(m_env_ptr != nullptr);
        assert(m_search_ptr != nullptr);
        m_search_ptr->force_planning_from_scratch();
        std::vector<int> soltn_ids;
        auto then = smpl::clock::now();
        bool success = m_search_ptr->replan( m_planning_time, &soltn_ids, &(_planner_soltn.cost) );
        double planning_time = smpl::to_seconds(smpl::clock::now() - then);
        _planner_soltn.planning_time = planning_time;
        _planner_soltn.num_expansions = m_search_ptr->m_num_expansions;

        if(!success){
            SMPL_ERROR("Planning failed.");
            return false;
        }

        m_env_ptr->extractPath( soltn_ids, _planner_soltn.robot_states );

        return true;
    }

} //namespace MPlanner

#endif
