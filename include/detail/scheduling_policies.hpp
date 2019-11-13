#ifndef WALKER_SCHEDULING_POLICIES_IMPLEMENTATION_H
#define WALKER_SCHEDULING_POLICIES_IMPLEMENTATION_H

#include "../scheduling_policies.h"

template <typename C>
ContextualDTSPolicy<C>::ContextualDTSPolicy( int _num_arms,
        int _num_contexts,
        unsigned int _seed ) :
    ContextualMABPolicy<C>(_num_arms),
    m_seed{_seed}
{
    m_alphas.resize(_num_contexts);
    m_betas.resize(_num_contexts);
    for( int i = 0; i < _num_contexts; i++ )
    {
        // No Prior
        m_alphas[i].resize(_num_arms, 1);
        m_betas[i].resize(_num_arms, 1);
    }
    srand(_seed);
    gsl_rng_env_setup();
    m_gsl_rand_T = gsl_rng_default;
    m_gsl_rand = gsl_rng_alloc(m_gsl_rand_T);
}

template <typename C>
ContextualDTSPolicy<C>::~ContextualDTSPolicy()
{
    gsl_rng_free(m_gsl_rand);
}

// Pass a negative rep_id if the queue was empty and hence did not have any
// element at the top.
template <typename C> int
ContextualDTSPolicy<C>::getArm( const std::vector<C>& _contexts, const std::vector<int>& _rep_ids )
{
    int N = _contexts.size();
    std::vector<int> context_ids(N, 0);
    for(auto& context : _contexts)
        context_ids.push_back(m_context_id_map[context]);

    std::vector<double> rep_likelihoods(N, 0);
    // Beta distribution for each representation
    double best_likelihood = -1;
    for( int i = 0; i < N; i++ )
    {
        int context_id = context_ids[i];
        int rep_id = _rep_ids[i];
        auto alpha_i = m_alphas[context_id][rep_id];
        auto beta_i = m_betas[context_id][rep_id];
        if(rep_id < 0)
            rep_likelihoods[i] = 0;
        else
            rep_likelihoods[i] = gsl_ran_beta(m_gsl_rand, alpha_i, beta_i);
        //doubledouble betaMean = alpha[i] / (alpha[i] + beta[i]);
        if(rep_likelihoods[i] > best_likelihood)
            best_likelihood = rep_likelihoods[i];
    }

    //because of quantization we can get the exact same random value
    //for multiple queues more often than we'd like
    //especially when beta is very low (we get 1 very easily)
    //or when alpha is very low (we get 0 very easily)
    //in these cases, there is a bias toward the lower index queues
    //because they "improve" best_rand first
    //so when there are multiple queues near the best_rand value,
    //we will choose uniformly at random from them
    std::vector<int> near_best_likelihood;
    for (int i = 0; i < N; i++)
    {
      if (fabs( best_likelihood - rep_likelihoods[i] ) < 0.0001) {
        near_best_likelihood.push_back(i);
      }
    }
    //ROS_ERROR("%f, %f, %f", rep_likelihoods[0], rep_likelihoods[1], rep_likelihoods[2]);
    //TODO: Take into account the branching factor of each rep.
    int best_id = near_best_likelihood[rand() % near_best_likelihood.size()];
    return best_id;
}

template <typename C> void
ContextualDTSPolicy<C>::updatePolicy(const C& _context, double _reward, int _arm)
{
    //ROS_ERROR("Context updating: ");
    //ROS_ERROR_STREAM(_context[0] << " "<< _context[1]<< " "<< _context[2]<< " "<< _context[3]<<"\n");
    int context_id = m_context_id_map[_context];
    //ROS_ERROR("Updating: ");
    //ROS_ERROR("  Context id: %d, Reward: %f, Arm: %d", context_id, _reward, _arm);
    if(_reward > 0)
        m_alphas[context_id][_arm] += 1;
    else
        m_betas[context_id][_arm] += 1;
    if( m_alphas[context_id][_arm] + m_betas[context_id][_arm] > m_C ){
        m_alphas[context_id][_arm] *= (m_C/(m_C + 1));
        m_betas[context_id][_arm] *= (m_C/(m_C + 1));
    }
}

template <typename C> bool
ContextualDTSPolicy<C>::setContextIdMap( const std::vector<C>& _contexts,
        const std::vector<int>& _rep_ids )
{
    if(_contexts.size() != _rep_ids.size())
        return false;
    if(_contexts.size() != m_alphas.size())
            return false;
    //ROS_ERROR("Context-id map");
    for( int i = 0; i < _contexts.size(); i++)
    {
        m_context_id_map[_contexts[i]] = _rep_ids[i];
        //ROS_ERROR("  Context: %d, %d, %d, %d", _contexts[i][0], _contexts[i][1], _contexts[i][2], _contexts[i][3]);
        //ROS_ERROR("  id: %d", _rep_ids[i]);
    }

    return true;
}

template <typename C> void
ContextualDTSPolicy<C>::setBetaPrior( const C& _context, int _rep_id, int _alpha, int _beta)
{
    int context_id = m_context_id_map[_context];
    m_alphas[context_id][_rep_id] = _alpha;
    m_betas[context_id][_rep_id] = _beta;
}

#endif
