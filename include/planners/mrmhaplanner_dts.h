#ifndef WALKER_MRMHAPLANNER_DTS_H
#define WALKER_MRMHAPLANNER_DTS_H

#include <algorithm>
#include <array>
// Beta distribution
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include <sbpl/planners/planner.h>
#include <sbpl/heuristics/heuristic.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/console/console.h>

#include "mrmhaplanner.h"

/**
 * N : Number of queues
 * R : Number of representations
 * SP: Scheduling Policy
 */
template <int N, int R, typename SP>
class MRMHAPlannerDTS : public MRMHAPlanner<N, R, SP> {
    public:
    MRMHAPlannerDTS(
        // Ideally, I should have a MultiRepDiscreteSpace
        DiscreteSpaceInformation* env,
        std::array<Heuristic*, N>& heurs,
        std::array<int, N>& rep_ids,
        std::array<std::array<int, R>, R>& rep_dependency_matrix,
        SP* scheduling_policy );

    virtual ~MRMHAPlannerDTS();

    /// Required from SBPLPlanner:
    //{
    virtual int replan(
            double allocated_time_sec,
            std::vector<int>* solution ) override;
    virtual int replan(
            double allocated_time_sec,
            std::vector<int>* solution,
            int* soltn_cost) override;
    virtual int replan(
            std::vector<int>* solution,
            ReplanParams params,
            int* soltn_cost) override;

    private:

    using MRMHASearchState = typename MRMHAPlanner<N, R, SP>::MRMHASearchState;
    using MRMHAPlanner<N, R, SP>::reinit_search;
    using MRMHAPlanner<N, R, SP>::reinit_state;
    using MRMHAPlanner<N, R, SP>::num_heuristics;
    using MRMHAPlanner<N, R, SP>::get_minf;
    using MRMHAPlanner<N, R, SP>::compute_key;
    using MRMHAPlanner<N, R, SP>::m_start_state;
    using MRMHAPlanner<N, R, SP>::m_goal_state;
    using MRMHAPlanner<N, R, SP>::m_rep_ids;
    using MRMHAPlanner<N, R, SP>::m_rep_dependency_matrix;
    using MRMHAPlanner<N, R, SP>::m_open;
    using MRMHAPlanner<N, R, SP>::m_params;
    using MRMHAPlanner<N, R, SP>::m_eps;
    using MRMHAPlanner<N, R, SP>::m_eps_mha;

    void updateDistribution(int ridx, int reward);
    virtual void expand(MRMHASearchState* state, int hidx);

    // 0 index reserved for the anchor queue
    // even if we won't be updating anchor's values.
    std::array<double, R> m_alphas {}, m_betas {} ;
    double m_C = 10;
    std::array<int, N> m_best_h, m_prev_best_h ;
    std::array<int, R> m_rep_h_count {};
    const gsl_rng_type* m_gsl_rand_T;
    gsl_rng* m_gsl_rand;

    int chooseRep();

};

#include "detail/mrmhaplanner_dts.hpp"

#endif
