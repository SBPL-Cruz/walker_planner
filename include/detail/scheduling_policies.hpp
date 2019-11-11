#ifndef WALKER_SCHEDULING_POLICIES_IMPLEMENTATION_H
#define WALKER_SCHEDULING_POLICIES_IMPLEMENTATION_H
#include "../scheduling_policies.h"

template <typename T>
DTSPolicy<T>::DTSPolicy( int _num_arms, unsigned int _seed ) :
    MABPolicy<T>(_num_arms),
    m_seed{_seed}
{
    m_alphas.resize(_num_arms, 1);
    m_betas.resize(_num_arms, 1);
    srand(_seed);
    gsl_rng_env_setup();
    m_gsl_rand_T = gsl_rng_default;
    m_gsl_rand = gsl_rng_alloc(m_gsl_rand_T);
}

template <typename T>
DTSPolicy<T>::~DTSPolicy()
{
    gsl_rng_free(m_gsl_rand);
}

template <typename T> int
DTSPolicy<T>::getAction()
{
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

template <typename T> void
DTSPolicy<T>::updatePolicy( T _reward, int _arm )
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

#endif
