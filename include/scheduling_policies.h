#include <stdlib.h>
#include <vector>
#include <utility>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include <smpl/types.h>
#include <smpl/heuristic/mother_heuristic.h>
#include <smpl/graph/manip_lattice_multi_rep.h>
#include <sbpl_collision_checking/shapes.h>
#include <sbpl/planners/scheduling_policy.h>
#include "utils/utils.h"

using CollisionObjects = std::vector<smpl::collision::CollisionObject>;

/*
class DoorFeaturePolicy {
    public:
    DoorFeaturePolicy(
            CollisionObjects//,
            )  {}
    inline std::vector<double> getFeatures(std::vector<double> _state){
        // Check the 2d/3d base path to see if it goes through any door.
        // Compute x, y distance from it.
        //
    }

    private:
    CollisionObjects m_doors;

    std::vector<std::string> m_feature_names = {
        "x_rel_door",
        "y_rel_door",
        "arm_retracted" // {0, 1} i.e. yes or no.
    };
}

template <typename FeaturePolicy>
class DecisionTreePolicy : public SchedulingPolicy {
    public:
    DecisionTreePolicy( int _num_queues,
            unsigned int _seed,
            const FeaturePolicy* _features_policy ) :
        SchedulingPolicy(_num_queues),
        m_features_policy_ptr{_features_policy} {
        srand(_seed);
    }

    inline virtual int getNextQueue(smpl::RobotState _s){
        auto features = m_features_policy_ptr->getFeatures(_s);

    }

    private:
    FeaturePolicy* m_features_policy_ptr;
    //std::unique_ptr<DecisionTree> m_decision_tree;
};
*/

class UniformlyRandomPolicy : public SchedulingPolicy {
    public:
        UniformlyRandomPolicy( int _num_queues, unsigned int _seed ) :
            SchedulingPolicy(_num_queues),
            m_seed{_seed}{
            srand(_seed);
        }
    //inline virtual int getNextQueue(const smpl::RobotState&) override;
        inline virtual double getActionSpaceProb(int state_id, int hidx){
            return 0.5;
        }

    private:
    unsigned int m_seed;
};

class RoundRobinPolicy : public SchedulingPolicy {
    public:
    RoundRobinPolicy(int num_queues) :
        SchedulingPolicy(num_queues) {}
    inline virtual double getActionSpaceProb(int state_id, int hidx){
        //std::cerr<<hidx - 1 <<" " <<numQueues()<< " " << m_queue<<"\n";
        if((hidx - 1) == m_queue){
            if(hidx == numQueues())
                m_queue = (m_queue + 1) % numQueues();
            return 1.0;
        } else {
            if(hidx == numQueues())
                m_queue = (m_queue + 1) % numQueues();
            return  0.0;
        }
    }

    private:
    int m_queue = 0;
    int m_iter = 0;
};

using Point = std::array<double, 2>;

class DirichletPolicy : public SchedulingPolicy {
    public:
    DirichletPolicy( int _num_queues, unsigned int _seed,
            smpl::ManipLatticeMultiRep* _manip_space_mr, BfsHeuristic* _base_heur, Point _door_loc ) :
        SchedulingPolicy(_num_queues),
        m_seed{_seed},
        m_manip_space_mr{_manip_space_mr},
        m_base_heur{_base_heur},
        m_door_loc{_door_loc} {
        srand(_seed);
        const gsl_rng_type* T;
        gsl_rng_env_setup();
        gsl_rng_default_seed = _seed;
        T = gsl_rng_default;
        m_gsl_rng = gsl_rng_alloc( T );
    }

    ~DirichletPolicy(){
        gsl_rng_free(m_gsl_rng);
    }

    /*
    inline virtual int getNextQueue(const smpl::RobotState& s) override {
        auto sqrd = [](double x){return x*x;};
        if(sqrt( sqrd(s[0] - m_door_loc.first) + sqrd(s[1] - m_door_loc.second) ) < m_thresh){

        }
    }
    */

    inline virtual double getActionSpaceProb( int state_id, int hidx ){
        //if state_id near goal
        if(state_id == 0)
            return 1.0;
        auto robot_state = m_manip_space_mr->getHashEntry(state_id)->state;
        double probs[9];
        if( m_base_heur->getMetricGoalDistance(robot_state[0], robot_state[1], 0) < 0.5){
            std::vector<double> conc_params = {50, 5, 5, 5, 5, 5, 5, 5, 5};
            gsl_ran_dirichlet( m_gsl_rng, 9, conc_params.data(), probs );
        }

        ////if state_id near door
        else if( euclidDist(robot_state.data(), m_door_loc.data(), 2) < 0.5 ){
            std::vector<double> conc_params = {50, 5, 5, 5, 5, 5, 5, 5, 5};
            gsl_ran_dirichlet( m_gsl_rng, 9, conc_params.data(), probs );
        }

        //otherwise: high prob on base and low on arm
        // 2 params
        else {
            std::vector<double> conc_params = {10, 10, 10, 10, 10, 10, 10, 10, 10};
            gsl_ran_dirichlet( m_gsl_rng, 9, conc_params.data(), probs );
        }
        //Ignoring anchor
        return probs[hidx-1];
    }

    private:
    smpl::ManipLatticeMultiRep* m_manip_space_mr;
    BfsHeuristic* m_base_heur;
    Point m_door_loc;
    double m_thresh = 0.8;
    unsigned int m_seed;
    gsl_rng* m_gsl_rng;
};
