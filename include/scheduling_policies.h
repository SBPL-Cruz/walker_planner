#ifndef WALKER_SCHEDULING_POLICIES_H
#define WALKER_SCHEDULING_POLICIES_H

#include <stdlib.h>
#include <vector>
#include <utility>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <boost/functional/hash.hpp>
#include <fstream>
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xview.hpp"

#include <smpl/types.h>
#include <smpl/heuristic/mother_heuristic.h>
#include <smpl/graph/manip_lattice_multi_rep.h>
#include <sbpl_collision_checking/shapes.h>
#include <sbpl/planners/scheduling_policy.h>
#include "utils/utils.h"

#include "config/planner_config.h"
#include "context_features.h"

using CollisionObjects = std::vector<smpl::collision::CollisionObject>;

class MetaMHAStarPolicy : SchedulingPolicy
{
    public:
    MetaMHAStarPolicy(int num_queues, std::vector<int> delta_h, std::vector<int> edge_costs, double w);
    ~MetaMHAStarPolicy()
    {}

     double getActionSpaceProb(int state_id, int hidx) override
     {
        throw "Not Implemented";
    }

    void initialize(std::vector<int>& start_h);
    int getAction() override;
    void updatePolicy(int hidx, int min_h);
    void updateMinH(int hidx, int min_h);
    void reset();

    private:
    std::vector<int> m_G;
    std::vector<int> m_H;
    std::vector<int> m_F;
    std::vector<int> m_delta_h;
    std::vector<int> m_edge_costs;

    double m_w = 1.0;
    int m_MULTIPLIER = 1;//000;

};

class MetaAStarPolicy : SchedulingPolicy
{
    public:
    MetaAStarPolicy(int num_queues, std::vector<int> delta_h, std::vector<int> edge_costs, double w);
    ~MetaAStarPolicy()
    {}

     double getActionSpaceProb(int state_id, int hidx) override
     {
        throw "Not Implemented";
    }

    void initialize(std::vector<int>& start_h);
    int getAction() override;
    void updatePolicy(int hidx, int min_h);
    void updateMinH(int hidx, int min_h);
    void reset();

    private:
    std::vector<int> m_G;
    std::vector<int> m_H;
    std::vector<int> m_F;
    std::vector<int> m_delta_h;
    std::vector<int> m_edge_costs;

    double m_w = 1.0;
    int m_MULTIPLIER = 1000;

};

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
    bool setBranchingFactor(std::vector<int>& b_factor)
    {
        for( int i = 0; i < b_factor.size(); i++ )
            m_b_factor[i] = b_factor[i];
    }

    bool setRepNumQueues(std::vector<int>& rep_num_queues)
    {
        for( int i = 0; i < rep_num_queues.size(); i++ )
            m_rep_num_queues[i] = rep_num_queues[i];
    }

    virtual void reset()
    {
        for(int i = 0; i < numArms(); i++)
        {
            m_alphas[i] = 1.0;
            m_betas[i] = 1.0;
        }
    }

    protected:
    unsigned int m_seed;
    std::vector<double> m_alphas {}, m_betas {} ;
    std::vector<int> m_b_factor {};
    std::vector<int> m_rep_num_queues {};
    double m_C = 5;
    const gsl_rng_type* m_gsl_rand_T;
    gsl_rng* m_gsl_rand;

    bool m_use_cached_rep = false;
    int m_cached_rep = -1;
    int m_cached_queue_id = 0;
};

class RepDTSPolicy : public DTSPolicy
{
    public:
    RepDTSPolicy(int num_reps, std::vector<int>& rep_ids, unsigned int seed);

    int getArm();
    void updatePolicy(double reward, int arm);
    void reset() override;

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
        // Each line corresponds to a pull.
        // Not need to store the number of tries separately.
        for(int i = 0; i < m_contexts.size(); i++)
        {
            //auto context = entry.first;
            auto context = m_contexts[i];
            for(auto& val : context)
            {
                stream<< val<< "\t";
            }
            //auto rewards = entry.second;
            //auto tries = m_context_arm_tries[context];
            auto rewards = m_rewards[i];
            for(auto i = 0; i < rewards.size(); i++)
            {
                stream<< rewards[i]<< "\t";
                //stream<< tries[i]<< "\t";
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

template <typename ContextArray>
class ContextualHumanPolicy : public ContextualMABPolicy<ContextArray>
{
    public:
    ContextualHumanPolicy( int num_arms, unsigned int seed) :
            ContextualMABPolicy<ContextArray>(num_arms),
            m_seed{seed}
    {
        srand(seed);
        gsl_rng_env_setup();
        m_gsl_rand_T = gsl_rng_default;
        m_gsl_rand = gsl_rng_alloc(m_gsl_rand_T);
    }
    ~ContextualHumanPolicy()
    {
        gsl_rng_free(m_gsl_rand);
    }

    void setSeed(unsigned int seed)
    {
        gsl_rng_set(m_gsl_rand, seed);
        srand(seed);
    }

    using ContextualMABPolicy<ContextArray>::getAction;
    int getAction(const std::vector<ContextArray>&)
    {
        throw "Not Implemented";
    }

    // The hand-designed policy goes here.
    int getArm( const std::vector<ContextArray>& context, const std::vector<int>& rep_ids )
    {
        std::vector<int> arm_ids, base_ids;
        std::vector<double> likelihoods(rep_ids.size(), 1);
        // Based on the distance to sampled base poses.
        for(int i = 0; i < rep_ids.size(); i++)
        {
            if(rep_ids[i] == (int)Fullbody)
                arm_ids.push_back(i);
            else
                base_ids.push_back(i);
        }

        // Base
        for(int i = 0; i < base_ids.size(); i++)
        {
            int id = base_ids[i];
            if(context[id][1] > 2 && context[id][2] > 2 )
                likelihoods[id] += 5;
            if(context[id][1] <= 2 || context[id][2] <= 2)
            {
                for(int j = 0; j < arm_ids.size(); j++)
                    likelihoods[arm_ids[j]]+=5;
                //likelihoods[id] /= 10;
            }
        }

        //Arm
        for(int i = 0; i < arm_ids.size(); i++)
        {
            int id = arm_ids[i];
            if(context[id][1] <= 2 || context[id][2] <= 2)// || context[id][0] <= 2 )
                //likelihoods[id] *= 10;
                likelihoods[id] += 5;
        }

        auto discrete_t = gsl_ran_discrete_preproc(likelihoods.size(), likelihoods.data());
        return gsl_ran_discrete(m_gsl_rand, discrete_t);
        //if(context[1] > 2 && context[2] > 2 )
            //return base_ids[rand() % base_ids.size()];
        //else
            //return arm_ids[rand() % arm_ids.size()];
    }

    void updatePolicy(double, int) override
    {
        throw "Not Implemented";
    }

    // Get reward for arm i and update distribution..
    void updatePolicy( const ContextArray&, double reward, int  arm )
    {
        // Eh!
    }

    void resetPrior()
    {
        //Eh!
    }

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

#include "detail/scheduling_policies.hpp"

#endif
