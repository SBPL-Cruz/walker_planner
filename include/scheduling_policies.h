#ifndef WALKER_SCHEDULING_POLICIES_H
#define WALKER_SCHEDULING_POLICIES_H

#include <stdlib.h>
#include <vector>
#include <utility>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <boost/functional/hash.hpp>
#include <fstream>

#include <smpl/types.h>
#include <smpl/heuristic/mother_heuristic.h>
#include <smpl/graph/manip_lattice_multi_rep.h>
#include <sbpl_collision_checking/shapes.h>
#include <sbpl/planners/scheduling_policy.h>
#include "utils/utils.h"

#include "context_features.h"

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

    inline virtual double getActionSpaceProb(int state_id, int hidx){
        return 0.5;
    }

    int getAction() override
    {
        throw "Not Implemented";
    }

    private:
    unsigned int m_seed;
};

class RoundRobinPolicy : public SchedulingPolicy {
    public:
    RoundRobinPolicy(int num_queues) :
        SchedulingPolicy(num_queues) {}
    inline virtual double getActionSpaceProb(int state_id, int hidx){
        throw "Not Implemented";
    }

    int getAction() override
    {
        int arm = m_queue;
        m_queue = (m_queue + 1) % numQueues();
        return arm;
    }

    private:
    int m_queue = 0;
    int m_iter = 0;
};


class MABPolicy : public SchedulingPolicy
{
    public:
    MABPolicy( int num_arms ) :
        SchedulingPolicy(num_arms) {}

    int numArms()
    {
        return this->numQueues();
    }

    double getActionSpaceProb( int state_id, int hidx ) override
    {
        throw "Not Implemented";
    }

    int getAction() = 0;

    virtual void updatePolicy( double reward, int arm ) = 0;
};

class DTSPolicy : public MABPolicy
{
    public:
    DTSPolicy( int num_arms, unsigned int seed );
    ~DTSPolicy();

    int getAction() override;
    int getArm() { return getAction(); }
    void updatePolicy( double reward, int arm );
    void setSeed(unsigned int seed)
    {
        gsl_rng_set(m_gsl_rand, seed);
    }

    protected:
    unsigned int m_seed;
    std::vector<double> m_alphas {}, m_betas {} ;
    double m_C = 10;
    const gsl_rng_type* m_gsl_rand_T;
    gsl_rng* m_gsl_rand;
};

class RepDTSPolicy : public DTSPolicy
{
    public:
    RepDTSPolicy(int num_reps, std::vector<int>& rep_ids, unsigned int seed);

    int getArm();
    void updatePolicy(double reward, int arm);

    private:
    std::vector< std::vector<int> > m_rep_hids;
    std::vector<int> m_rep_ids;

};

class UCBPolicy : public MABPolicy
{
    public:
    UCBPolicy( int num_arms, unsigned int seed );

    int getAction() override;
    void updatePolicy( double reward, int arm ) override;

    private:
    double m_alpha = 0.75;
    int m_T = 0;
    std::vector<int> m_pull_counts;
    std::vector<double> m_rewards;

    std::vector< std::vector<double> > m_ucbs;
    std::vector<int> m_pulls;
};

template <typename ContextArray>
class ContextualMABPolicy : public MABPolicy
{
    public:
    ContextualMABPolicy( int num_arms ) :
        MABPolicy(num_arms) {}

    virtual int getAction( const std::vector<ContextArray>& ) = 0;

    int getAction() override
    {
        throw "Not Implemented";
    }

    void updatePolicy( double, int ) = 0;
};

template <class T>
struct ContextArrayHash
{
    size_t operator() (const T& v) const
    {
        std::size_t seed = 0;
        for(auto& val : v)
        {
            boost::hash_combine(seed, std::hash<typename T::value_type>()(val));
        }
        return seed;
    }
};

template <typename ContextArray>
class ContextualDTSPolicy : public ContextualMABPolicy<ContextArray>
{
    public:
    ContextualDTSPolicy( int num_arms, unsigned int seed);
    ~ContextualDTSPolicy();

    using ContextualMABPolicy<ContextArray>::getAction;
    int getAction(const std::vector<ContextArray>&)
    {
        throw "Not Implemented";
    }

    void setSeed(unsigned int seed)
    {
        gsl_rng_set(m_gsl_rand, seed);
    }

    int getContextId(ContextArray);

    //Get the context-array for the state at the top of every inadmissible
    //queue and the queue's rep-id.
    // Pick the right beta distributions on that basis and then TS.
    int getAction( const std::vector<ContextArray>&, const std::vector<int>& rep_ids ){}
    int getArm( const std::vector<ContextArray>&, const std::vector<int>& rep_ids );

    void updatePolicy(double, int) override 
    {
        throw "Not Implemented";
    }

    // Get reward for arm i and update distribution..
    void updatePolicy( const ContextArray&, double reward, int  arm );

    //bool setContextIdMap( const std::vector<ContextArray>&, const std::vector<int>& );
    //void setBetaPrior( const ContextArray& context, int rep_id, int alpha, int beta );
    void resetPrior()
    {
        //for(int i = 0; i < this->numArms(); i++)
            //m_alphas[-1][i] = 1;
    }

    bool loadBetaPrior(std::string file_name);

    private:
    unsigned int m_seed;
    std::unordered_map< ContextArray, int, ContextArrayHash<ContextArray> > m_context_id_map;
    std::unordered_map<int, std::vector<double> > m_alphas, m_betas;
    //std::vector< std::vector<double> > m_alphas {}, m_betas {} ;
    int m_min_C = 10;
    int m_max_C = 60;//std::numeric_limits<int>::max();
    std::unordered_map<int, std::vector<int> > m_C_map;
    const gsl_rng_type* m_gsl_rand_T;
    gsl_rng* m_gsl_rand;
};


template <typename ContextArray>
class ContextualMABTrainPolicy : public ContextualMABPolicy<ContextArray>
{
    public:
    ContextualMABTrainPolicy( int num_reps, int num_queues, unsigned int seed ) :
        ContextualMABPolicy<ContextArray>(num_reps),
        m_num_queues{num_queues}{}

    ~ContextualMABTrainPolicy() {}

    int getArm( const std::vector<ContextArray>&, const std::vector<int>& rep_ids )
    {
        return getAction();
    }
    int getAction() override
    {
        //int arm = m_queue;
        //m_queue = (m_queue + 1) % m_num_queues;
        //return arm;
        return rand() % m_num_queues;
    }

    int getAction(const std::vector<ContextArray>&)
    {
        throw "Not Implemented";
    }

    void updatePolicy( double, int )
    {
        throw "Not Implemented";
    }

    void updatePolicy( const ContextArray& context, double reward, int arm )
    {
        if(  m_context_arm_reward.count(context) == 0 )
        {
            std::vector<double> rewards(this->numArms(), 0);
            m_context_arm_reward[context] = std::move(rewards);
        }
        m_context_arm_reward[context][arm] += reward;

        // Ensures every reward is 1 or 0.
        m_contexts.push_back(context);
        std::vector<double> arm_reward(2, 0);
        arm_reward[0] = arm;
        arm_reward[1] = reward;
        m_rewards.push_back(arm_reward);
    }

    bool writeToFile(std::string file_name = "context_rewards.txt")
    {
        ROS_ERROR("Size of context: %d", m_context_arm_reward.size());
        std::ofstream stream;
        stream.open(file_name, std::ios::app);
        if(!stream.is_open())
        {
            ROS_ERROR("Could not open context file to write to.");
            return false;
        }
        //for(auto& entry : m_context_arm_reward)
        for(int i = 0; i < m_contexts.size(); i++)
        {
            //auto context = entry.first;
            auto context = m_contexts[i];
            for(auto& val : context)
            {
                stream<< val<< "\t";
            }
            //auto rewards = entry.second;
            auto rewards = m_rewards[i];
            for(auto i = 0; i < rewards.size(); i++)
            {
                stream<< rewards[i]<< "\t";
            }
            stream<< "\n";
        }
        stream.close();
        return true;
    }

    void clear()
    {
        m_context_arm_reward.clear();
        m_contexts.clear();
        m_rewards.clear();
    }

    private:
    int m_queue = 0;
    int m_iter = 0;
    int m_num_queues = 0;

    // Context is row, arm is column and reward is value.
    std::unordered_map< ContextArray, std::vector<double>,
        ContextArrayHash<ContextArray> > m_context_arm_reward;
    std::vector<ContextArray> m_contexts;
    std::vector< std::vector<double> > m_rewards;
};

//using Point = std::array<double, 2>;

//class DirichletPolicy : public SchedulingPolicy {
    //public:
    //DirichletPolicy( int _num_queues, unsigned int _seed,
            //smpl::ManipLatticeMultiRep* _manip_space_mr, smpl::CompoundBfsHeuristic* _base_heur, Point _door_loc ) :
        //SchedulingPolicy(_num_queues),
        //m_seed{_seed},
        //m_manip_space_mr{_manip_space_mr},
        //m_base_heur{_base_heur},
        //m_door_loc{_door_loc} {
        //srand(_seed);
        //const gsl_rng_type* T;
        //gsl_rng_env_setup();
        //gsl_rng_default_seed = _seed;
        //T = gsl_rng_default;
        //m_gsl_rng = gsl_rng_alloc( T );
    //}

    //~DirichletPolicy(){
        //gsl_rng_free(m_gsl_rng);
    //}

    //[>
    //inline virtual int getNextQueue(const smpl::RobotState& s) override {
        //auto sqrd = [](double x){return x*x;};
        //if(sqrt( sqrd(s[0] - m_door_loc.first) + sqrd(s[1] - m_door_loc.second) ) < m_thresh){

        //}
    //}
    //*/

    //inline virtual double getActionSpaceProb( int state_id, int hidx ){
        ////if state_id near goal
        //if(state_id == 0)
            //return 1.0;
        //auto robot_state = m_manip_space_mr->getHashEntry(state_id)->state;
        //double probs[9];
        //if( m_base_heur->getMetricGoalDistance(robot_state[0], robot_state[1], 0) < 0.5){
            //std::vector<double> conc_params = {50, 5, 5, 5, 5, 5, 5, 5, 5};
            //gsl_ran_dirichlet( m_gsl_rng, 9, conc_params.data(), probs );
        //}

        //////if state_id near door
        //else if( euclidDist(robot_state.data(), m_door_loc.data(), 2) < 0.5 ){
            //std::vector<double> conc_params = {50, 5, 5, 5, 5, 5, 5, 5, 5};
            //gsl_ran_dirichlet( m_gsl_rng, 9, conc_params.data(), probs );
        //}

        ////otherwise: high prob on base and low on arm
        //// 2 params
        //else {
            //std::vector<double> conc_params = {10, 10, 10, 10, 10, 10, 10, 10, 10};
            //gsl_ran_dirichlet( m_gsl_rng, 9, conc_params.data(), probs );
        //}
        ////Ignoring anchor
        //return probs[hidx-1];
    //}

    //private:
    //smpl::ManipLatticeMultiRep* m_manip_space_mr;
    //smpl::CompoundBfsHeuristic* m_base_heur;
    //Point m_door_loc;
    //double m_thresh = 0.8;
    //unsigned int m_seed;
    //gsl_rng* m_gsl_rng;
 //};

#include "detail/scheduling_policies.hpp"

#endif
