#include "scheduling_policies.h"
#include <algorithm>
#include <panini/algo.h>
#include <cmath>

#define INFINITECOST std::numeric_limits<int>::max()

#define SAMPLING_LOG "dts.sampling"
#define UPDATE_LOG "policy.update"
#define PARAMS_LOG "policy.params"

MetaAStarPolicy::MetaAStarPolicy(int _num_queues, std::vector<int> _delta_h, std::vector<double> _edge_costs, double _w) :
    SchedulingPolicy(_num_queues),
    m_delta_h{_delta_h},
    m_w{_w}
{
    for(auto val : _edge_costs)
        m_edge_costs.push_back(m_MULTIPLIER * val);
    ROS_DEBUG_NAMED(PARAMS_LOG, "  Edge Cost: %d, %d, %d, %d", m_edge_costs[0], m_edge_costs[1], m_edge_costs[2], m_edge_costs[3]);
    m_G.resize(_num_queues, 0);
    m_H.resize(_num_queues, INFINITECOST);
    m_F.resize(_num_queues, INFINITECOST);
}

void MetaAStarPolicy::initialize(std::vector<int>& _start_h)
{
    assert(_start_h.size() == numQueues());
    ROS_DEBUG_NAMED(PARAMS_LOG, "Initializing: ");
    for(int i = 0; i < _start_h.size(); i++)
    {
        m_H[i] = m_MULTIPLIER*( _start_h[i] / m_delta_h[i] );
        m_F[i] = m_w * m_H[i];
        ROS_DEBUG_NAMED(PARAMS_LOG, "  id: %d, H: %d, F: %d", i, m_H[i], m_F[i]);
    }
}

int MetaAStarPolicy::getAction()
{
    ROS_DEBUG_NAMED(PARAMS_LOG, "h0: %d h1: %d h2: %d", m_H[0], m_H[1], m_H[2]);
    int min_idx = 0;
    for(int i = 0; i < numQueues(); i++)
    {
        if(m_F[i] < m_F[min_idx] && m_H[i] > 0)
            min_idx = i;
    }
    return min_idx;
}

void MetaAStarPolicy::updatePolicy(int _hidx, int _min_h)
{
    assert(_hidx < numQueues());
    m_G[_hidx] += m_edge_costs[_hidx];//1;
    m_H[_hidx] = m_MULTIPLIER*(_min_h / m_delta_h[_hidx]);
    m_F[_hidx] = m_G[_hidx] + m_w * m_H[_hidx];
    ROS_DEBUG_NAMED(UPDATE_LOG, " Meta A*: Queue: %d  F: %d G: %d H: %d", _hidx, m_F[_hidx], m_G[_hidx], m_H[_hidx]);
}

void MetaAStarPolicy::updateMinH(int _hidx, int _min_h)
{
    assert(_hidx < numQueues());
    m_H[_hidx] = m_MULTIPLIER*(_min_h / m_delta_h[_hidx]);
    m_F[_hidx] = m_G[_hidx] + m_w * m_H[_hidx];
    ROS_DEBUG_NAMED(UPDATE_LOG, " Meta A*: Queue: %d  F: %d G: %d H: %d", _hidx, m_F[_hidx], m_G[_hidx], m_H[_hidx]);
}

void MetaAStarPolicy::reset()
{
    m_G.resize(numQueues(), 0);
    m_H.resize(numQueues(), INFINITECOST);
    m_F.resize(numQueues(), INFINITECOST);
}

DTSPolicy::DTSPolicy( int _num_arms, unsigned int _seed ) :
    MABPolicy(_num_arms),
    m_seed{_seed}
{
    m_alphas.resize(_num_arms, 1);
    m_betas.resize(_num_arms, 1);
    m_b_factor.resize(_num_arms, 1);
    m_rep_num_queues.resize(_num_arms, 1);
    srand(_seed);
    gsl_rng_env_setup();
    m_gsl_rand_T = gsl_rng_default;
    m_gsl_rand = gsl_rng_alloc(m_gsl_rand_T);
}

DTSPolicy::~DTSPolicy()
{
    gsl_rng_free(m_gsl_rand);
}

int DTSPolicy::getAction() {
    //ROS_ERROR("alphas: %f, %f", m_alphas[0], m_alphas[1]);
    std::vector<double> rep_likelihoods(this->numArms(), 0);
    // Beta distribution for each representation
    double best_likelihood = -1;
    for( int i = 0; i < this->numArms(); i++ )
    {
        //rep_likelihoods[i] = gsl_ran_beta(m_gsl_rand, m_alphas[i], m_betas[i]);
        rep_likelihoods[i] = gsl_ran_beta(m_gsl_rand, m_alphas[i], m_betas[i]);
        //rep_likelihoods[i] /= m_b_factor[i]; // Expand more from rep with smaller branching factor
        //rep_likelihoods[i] *= m_rep_num_queues[i]; //Expand more from rep with more queues.
        //double betaMean = alpha[i] / (alpha[i] + beta[i]);
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
    for (int i = 0; i < this->numArms(); i++)
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

void DTSPolicy::updatePolicy( double _reward, int _arm )
{
    if(_reward > 0)
        m_alphas[_arm] += 1;
    else
        m_betas[_arm] += 1;
    if( m_alphas[_arm] + m_betas[_arm] > m_C ){
        m_alphas[_arm] *= (m_C/(m_C + 1));
        m_betas[_arm] *= (m_C/(m_C + 1));
    }
    m_alphas[_arm] = ceil(m_alphas[_arm]);
    m_betas[_arm] = ceil(m_betas[_arm]);
    ROS_DEBUG_NAMED(PARAMS_LOG, "    arm: %d, params(alpha, beta, mean): %f, %f, %f", _arm, m_alphas[_arm], m_betas[_arm], m_alphas[_arm] / ( m_alphas[_arm] + m_betas[_arm] ));
    //ros::Duration(0.5).sleep();
}

RepDTSPolicy::RepDTSPolicy(int _num_reps, std::vector<int>& _rep_ids, unsigned int _seed) :
    DTSPolicy(_num_reps, _seed)
{
    m_rep_hids.resize(_num_reps);
    for(int i = 1; i < _rep_ids.size(); i++)
    {
        m_rep_hids[_rep_ids[i]].push_back(i);
    }
    m_rep_ids = _rep_ids;//std::vector<int>(_rep_ids.begin() + 1, _rep_ids.end());
    // Skip rep if no queue corresponding to it.
    for(int i = 0; i < _num_reps; i++)
    {
        if(!m_rep_hids[i].size())
        {
            DTSPolicy::m_alphas[i] = 0;
            DTSPolicy::m_betas[i] = 0;
        }
    }
}

int RepDTSPolicy::getArm()
{
    //if(not m_use_cached_rep)
    //{
        //m_cached_rep = DTSPolicy::getAction();
        //m_cached_queue_id = 0;
        //m_use_cached_rep = true;
    //}
    //int queue_id = m_rep_hids[m_cached_rep][m_cached_queue_id];
    //m_cached_queue_id++;
    //if(m_cached_queue_id >= m_rep_hids[m_cached_rep].size())
        //m_use_cached_rep = false;
    //return queue_id - 1;

    // Returns a random queue in this rep id.
    int rep = DTSPolicy::getAction();
    int id = rand() % m_rep_hids[rep].size();
     //hidx is incremented by 1 in the planner
    return m_rep_hids[rep][id] - 1;
}

void RepDTSPolicy::updatePolicy(double _reward, int _hidx)
{
    DTSPolicy::updatePolicy(_reward, m_rep_ids[_hidx]);
}

void RepDTSPolicy::reset()
{
    for(int i = 0; i < numArms(); i++)
    {
        if(!m_rep_hids[i].size())
        {
            DTSPolicy::m_alphas[i] = 0;
            DTSPolicy::m_betas[i] = 0;
        } else
        {
            DTSPolicy::m_alphas[i] = 1.0;
            DTSPolicy::m_betas[i] = 1.0;
        }
    }
}

UCBPolicy::UCBPolicy( int _num_arms, unsigned int _seed ) :
    MABPolicy(_num_arms),
    m_T{_num_arms}
{
    m_pull_counts.resize(_num_arms, 0);
    m_rewards.resize(_num_arms, 0);
    m_ucbs.resize(_num_arms);
    m_pulls.resize(_num_arms, 0);
    srand(_seed);
}

int UCBPolicy::getAction()
{
    m_T++;
    for( int i = 0; i < numArms(); i++ )
    {
        if( m_pulls[i] == 0 )
            return i;
    }
    std::vector<double> ucbs;
    for( int i = 0; i < numArms(); i++ )
    {
        double ucb = m_rewards[i] / m_pulls[i] +
                m_alpha* sqrt( 2*log(m_T) / m_pulls[i] );
        ucbs.push_back(ucb);
        m_ucbs[i].push_back(ucb);
    }
    int pull = panini::algo::argmax(ucbs);
    m_pulls[pull]++;
    return pull;
}

void UCBPolicy::updatePolicy( double _reward, int _hidx )
{
    m_rewards[_hidx] += _reward;
    m_pulls[_hidx] += 1;
}
