#ifndef MOTION_PLANNERS_H
#define MOTION_PLANNERS_H

#include <iostream>

namespace MPlanner {

    struct PlannerSolution {
        std::vector<smpl::RobotState> robot_states;
        int cost;
    }
}

namespace MPlanner {

    template <typename Search, typename Env>
        class MotionPlanner : public Search, public Env {
            public:
            MotionPlanner();
            ~MotionPlanner();

            virtual bool init(std::unique_ptr<Env> _env,
                    std::unique_ptr<Heuristic> _anchor, std::vector<std::unique_ptr<Heuristic>> m_inad );

            void set_initial_mha_eps(int _eps) {
                m_search_ptr->set_initial_mha_eps(_eps);
            }

            void set_initialsolution_eps(int _eps) {
                m_search_ptr->set_initialsolution_eps(_eps);
            }

            void set_search_mode(bool _mode) {
                m_search_ptr->set_search_mode(_mode);
            }

            virtual bool updateStart(moveit_msgs::RobotState&);
            virtual bool updateGoal(const smpl::GoalConstraint&);
            virtual bool plan(double _time, PlannerSolution& planner_soltn );

            private:
            std::unique_ptr<Search> m_search_ptr = nullptr;
            std::shared_ptr<Env> m_env_ptr = nullptr;
            std::unique_ptr<Heuristic> m_anchor_heur_ptr = nullptr;
            std::vector<std::unique_ptr<Heuristic>> m_inad_heurs;

            moveit_msgs::RobotState m_start_state;
            smpl::GoalConstraint m_goal_constraints;
        };

} //namespace MPlanner

namespace MPlanner {

    template <typename Search, typename Env>
        MotionPlanner<Search, Env>::MotionPlanner(){
            SMPL_INFO("MotionPlanner Constructor");
        }

    template <typename Search, typename Env>
        bool MotionPlanner<Search, Env>::init(
                std::unique_ptr<Env> _env_ptr,
                std::unique_ptr<Heuristic> _anchor_ptr,
                std::vector<std::unique_ptr<Heuristic>> _inad_heurs ) :
            m_env_ptr{_env_ptr}, m_anchor_heur_ptr{_anchor_ptr}, m_inad_heurs{_inad_heurs} {
            SMPL_INFO("MotionPlanner::init");

            // Implicit Requirement from Search Class
            m_search_ptr = make_unique<Search>( m_env_ptr.get(),
                    m_anchor_heur_ptr.get(), m_inad_heurs.data(), m_inad_heurs.size() );
        }

    template <typename Search, typename Env>
        bool MotionPlanner<Search, Env>::updateStart(
                const moveit_msgs::RobotState& _start_state ) :
            m_start_state{_start_state} {
            SMPL_INFO("updateStart");
        }

    template <typename Search, typename Env>
        bool MotionPlanner<Search, Env>::updateGoal(
                const smpl::GoalConstraint& _goal_constraint) :
            m_goal_constraints{_goal_constraint} {
            SMPL_INFO("updateGoal");
        }

    template <typename Search, typename Env>
        bool MotionPlanner<Search, Env>::plan( double _time, PlannerSolution& _planner_soltn ){
            SMPL_INFO("plan");
            m_search_ptr->force_planning_from_scratch();
            std::vector<int> soltn_ids;
            bool success = m_search_ptr->replan( _time, &soltn_ids, &(_planner_soltn.cost) );

            if(!success){
                SMPL_ERROR("Planning failed.");
                return false;
            }

            m_search_ptr->extractPath( soltn_ids, _planner_soltn.robot_states );

            return true;
        }

} //namespace MPlanner

#endif
