#include "scheduling_policies.h"
#include <panini/algo.h>

DTSPolicy::DTSPolicy( int _num_arms, unsigned int _seed ) :
    MABPolicy(_num_arms),
    m_seed{_seed}
{
    m_alphas.resize(_num_arms, 1);
    m_betas.resize(_num_arms, 1);
    srand(_seed);
    gsl_rng_env_setup();
    m_gsl_rand_T = gsl_rng_default;
    m_gsl_rand = gsl_rng_alloc(m_gsl_rand_T);
}

DTSPolicy::~DTSPolicy()
{
    gsl_rng_free(m_gsl_rand);
}

int DTSPolicy::getAction()
{
    //ROS_ERROR("alphas: %f, %f", m_alphas[0], m_alphas[1]);
    std::vector<double> rep_likelihoods(this->numArms(), 0);
    // Beta distribution for each representation
    double best_likelihood = -1;
    for( int i = 0; i < this->numArms(); i++ )
    {
        rep_likelihoods[i] = gsl_ran_beta(m_gsl_rand, m_alphas[i], m_betas[i]);
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
    int rep = DTSPolicy::getAction();
    int id = rand() % m_rep_hids[rep].size();
    // hidx is incremented by 1 in the planner
    return m_rep_hids[rep][id] - 1;
}

void RepDTSPolicy::updatePolicy(double _reward, int _hidx)
{
    DTSPolicy::updatePolicy(_reward, m_rep_ids[_hidx]);
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
