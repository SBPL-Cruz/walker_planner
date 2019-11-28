#ifndef WALKER_SCHEDULING_POLICIES_IMPLEMENTATION_H
#define WALKER_SCHEDULING_POLICIES_IMPLEMENTATION_H

#include <fstream>
#include <algorithm>
#include <cmath>
#include "../scheduling_policies.h"

#define SAMPLING_LOG "dts.sampling"
#define UPDATE_LOG "dts.update"
#define PARAMS_LOG "dts.params"

template <typename C>
ContextualDTSPolicy<C>::ContextualDTSPolicy( int _num_arms,
        unsigned int _seed ) :
    ContextualMABPolicy<C>(_num_arms),
    m_seed{_seed}
{
    //m_alphas.resize(_num_contexts);
    //m_betas.resize(_num_contexts);
    //for( int i = 0; i < _num_contexts; i++ )
    //{
        //// No Prior
        //m_alphas[i].resize(_num_arms, 1);
        //m_betas[i].resize(_num_arms, 1);
    //}
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

template <typename C> int
ContextualDTSPolicy<C>::getContextId(C _context)
{
    if(m_context_id_map.count(_context))
        return m_context_id_map[_context];
    else
    {
        auto id = m_context_id_map.size();
        m_context_id_map[_context] = id;
        std::vector<double> alpha(this->numArms(), 1);
        std::vector<double> beta(this->numArms(), 1);
        m_alphas[id] = alpha;
        m_betas[id] = beta;
        return id;
    }
}

// Pass a negative rep_id if the queue was empty and hence did not have any
// element at the top.
template <typename C> int
ContextualDTSPolicy<C>::getArm( const std::vector<C>& _contexts, const std::vector<int>& _rep_ids )
{
    int N = _contexts.size();
    std::vector<int> context_ids;
    for(auto& context : _contexts){
        //context_ids.push_back(m_context_id_map[context]);
        context_ids.push_back(getContextId(context));
        ROS_DEBUG_NAMED(SAMPLING_LOG, "Context: %d %d %d %d, id: %d", context[0], context[1], context[2], context[3], context_ids.back() );
    }

    std::vector<double> rep_likelihoods(N, 0);
    // Beta distribution for each representation
    double best_likelihood = -1;
    for( int i = 0; i < N; i++ )
    {
        int context_id = context_ids[i];
        int rep_id = _rep_ids[i];
        auto alpha_i = m_alphas[context_id][rep_id];
        auto beta_i = m_betas[context_id][rep_id];
        ROS_DEBUG_NAMED(PARAMS_LOG, "   context: %d, rep: %d, params: %f, %f", context_id, rep_id, alpha_i, beta_i);
        if(rep_id < 0)
            rep_likelihoods[i] = 0;
        else
            rep_likelihoods[i] = gsl_ran_beta(m_gsl_rand, alpha_i, beta_i);
        double betaMean = alpha_i / (alpha_i + beta_i);
        //ROS_INFO("h: %d, rep: %d, context: %d, alpha: %f, beta: %f, likelihood: %f, mean: %f", i, rep_id, context_id, alpha_i, beta_i, rep_likelihoods[i], betaMean);
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
    ROS_DEBUG_NAMED(SAMPLING_LOG, "Chosen h: %d, rep: %d", best_id, _rep_ids[best_id] );

    return best_id;
}

template <typename C> void
ContextualDTSPolicy<C>::updatePolicy(const C& _context, double _reward, int _arm)
{
    //ROS_DEBUG_NAMED(UPDATE_LOG, "Rep: %d, context: %d, %d, %d, %d", _arm, _context[0], _context[1], _context[2], _context[3]);
    //if(m_context_id_map.count(_context) == 0)
    //{
        //ROS_DEBUG_NAMED(UPDATE_LOG, "New context added: %d, %d, %d, %d", _context[0], _context[1], _context[2], _context[3] );
        //std::vector<int> v(this->numArms(), 1);
        //m_context_id_map[_context] = m_context_id_map.size();
    //}
    //int context_id = m_context_id_map[_context];
    int context_id = getContextId(_context);
    if(m_C_map.count(context_id) == 0){
        std::vector<int> cs(this->numArms(), m_min_C);
        m_C_map[context_id] = cs;
    }
    //auto dts_C = m_C_map[context_id][_arm];
    int dts_C = 0;
    //if(context_id == -1)
        //dts_C = m_min_C;
    //else
    dts_C = m_C_map[context_id][_arm];//m_max_C;
    ROS_DEBUG_NAMED(UPDATE_LOG, "Updating: ");
    ROS_DEBUG_NAMED(UPDATE_LOG, "  Context id: %d, Reward: %f, Arm: %d", context_id, _reward, _arm);
    ROS_DEBUG_NAMED(UPDATE_LOG, "  Before: %f, %f", m_alphas[context_id][_arm], m_betas[context_id][_arm]);
    if(_reward > 0)
        m_alphas[context_id][_arm] += 1;
    else
        m_betas[context_id][_arm] += 1;
    if( m_alphas[context_id][_arm] + m_betas[context_id][_arm] > dts_C ){
        auto sum = m_alphas[context_id][_arm] + m_betas[context_id][_arm];
        // Ensures that post update, alpha + beta = C
        m_alphas[context_id][_arm] *= ((double)dts_C/(sum));
        m_alphas[context_id][_arm] = std::max(1.0, (m_alphas[context_id][_arm]) );
        m_betas[context_id][_arm] *= ((double)dts_C/(sum));
        m_betas[context_id][_arm] = std::max( 1.0, (m_betas[context_id][_arm]) );
    }
    ROS_DEBUG_NAMED(UPDATE_LOG, "  After: %f, %f, C: %d", m_alphas[context_id][_arm], m_betas[context_id][_arm], dts_C);
    //ros::Duration(0.1).sleep();
    // Upate the prior-less params.
    // Supposed to track contex-independent local rewards.
    //context_id = -1;
    //if(_reward > 0)
        //m_alphas[context_id][_arm] += 1;
    //else
        //m_betas[context_id][_arm] += 1;
    //if( m_alphas[context_id][_arm] + m_betas[context_id][_arm] > dts_C ){
        //auto sum = m_alphas[context_id][_arm] + m_betas[context_id][_arm];
        //m_alphas[context_id][_arm] = std::max(1.0, (m_alphas[context_id][_arm]) );
        //m_alphas[context_id][_arm] *= ((double)dts_C/(sum));
        //// Ensures that post update, alpha + beta = C
        //m_betas[context_id][_arm] *= ((double)dts_C/(sum));
        //m_betas[context_id][_arm] = std::max( 1.0, (m_betas[context_id][_arm]) );
    //}
}

//template <typename C> bool
//ContextualDTSPolicy<C>::setContextIdMap( const std::vector<C>& _contexts,
        //const std::vector<int>& _rep_ids )
//{
    //if(_contexts.size() != _rep_ids.size())
        //return false;
    //if(_contexts.size() != m_alphas.size())
            //return false;
    ////ROS_ERROR("Context-id map");
    //for( int i = 0; i < _contexts.size(); i++)
    //{
        //m_context_id_map[_contexts[i]] = _rep_ids[i];
        ////ROS_ERROR("  Context: %d, %d, %d, %d", _contexts[i][0], _contexts[i][1], _contexts[i][2], _contexts[i][3]);
        ////ROS_ERROR("  id: %d", _rep_ids[i]);
    //}

    //return true;
//}

//template <typename C> void
//ContextualDTSPolicy<C>::setBetaPrior( const C& _context, int _rep_id, int _alpha, int _beta)
//{
    //int context_id = m_context_id_map[_context];
    //m_alphas[context_id][_rep_id] = _alpha;
    //m_betas[context_id][_rep_id] = _beta;
//}

template <typename C> bool
ContextualDTSPolicy<C>::loadBetaPrior(std::string _file_name)
{
    std::ifstream stream;
    stream.open(_file_name, std::ios::in);
    if(!stream.is_open()){
        ROS_ERROR("Could not open file to load beta prior.");
        return false;
    }
    int num_arms = this->numArms();
    std::string line;
    int i = 0;
    while(std::getline(stream, line))
    {
        std::istringstream iss(line);
        int val;
        std::vector<int> vals;
        while(iss>> val)
        {
            vals.push_back(val);
        }

        //ROS_WARN("Size of line: %d", vals.size());
        //ROS_WARN("Num of arms: %d", num_arms);
        //assert( vals.size() == ( C::size_type + 2*num_arms) );
        C context;
        int idx = 0;
        for(auto it = vals.begin(); it != vals.end() - 2*num_arms; it++)
        {
            context[idx++] = (double)*it;
        }
        std::vector<int> params(vals.end() - 2*num_arms, vals.end());
        m_context_id_map[context] = i;
        std::vector<double> alphas;
        std::vector<double> betas;
        std::vector<int> dts_C;
        for(int j = 0; j < 2*num_arms; j++)
        {
            if(j % 2)
            {
                betas.push_back((double)params[j]);
                //dts_C.back() += params[j];

            }
            else
            {
                alphas.push_back((double)params[j]);
                //dts_C.push_back((double)params[j]);
            }
            dts_C.push_back(m_max_C);
        }
        m_alphas[i] = alphas;
        m_betas[i] = betas;
        //int min = *std::min_element(dts_C.begin(), dts_C.end());
        //std::vector<int> common_C(dts_C.size(), min);
        //m_C_map[i] = common_C;
        m_C_map[i] = dts_C;
        i++;
    }

    // Set uniform prior for all contexts no loaded
    // Represent them by the id -1
    //std::vector<double> alphas(2*num_arms, 1.0);
    //std::vector<double> betas(2*num_arms, 1.0);
    //m_alphas[-1] = alphas;
    //m_betas[-1] = betas;
    ROS_WARN("Number of contexts: %d", m_context_id_map.size());

    stream.close();
    return true;
}

#endif
