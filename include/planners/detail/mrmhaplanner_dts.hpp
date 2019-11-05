#ifndef WALKER_MHAPLNANNER_DTS_IMPLEMENTATION_H
#define WALKER_MHAPLNANNER_DTS_IMPLEMENTATION_H

#include "../mrmhaplanner_dts.h"
#include <panini/maths.h>
#include <cmath>
#define LOG "mrmha_dts"

template <int N, int R, typename SP>
MRMHAPlannerDTS<N, R, SP>::MRMHAPlannerDTS(
        DiscreteSpaceInformation* _env,
        std::array<Heuristic*, N>& _heurs,
        std::array<int, N>& _rep_ids,
        std::array<std::array<int, R>, R>& _rep_dependency_matrix,
        SP* _scheduling_policy ) :
    MRMHAPlanner<N, R, SP>( _env, _heurs, _rep_ids, _rep_dependency_matrix, _scheduling_policy ){
    for(int i = 0; i < N; i++){
        m_alphas[i] = 1.0;
        m_betas[i] = 1.0;
        m_prev_best_h[i] = INFINITECOST;
    }
}

template <int N, int R, typename SP>
int MRMHAPlannerDTS<N, R, SP>::chooseRep(){
    for( int  i = 1; i < N; i++ ){
        if( m_best_h[i] < m_prev_best_h[i] )
            m_alphas[i] += 1;
        else
            m_betas[i] += 1;
        if( m_alphas[i] + m_betas[i] > m_C ){
            m_alphas[i] *= (m_C/(m_C + 1));
            m_betas[i] *= (m_C/(m_C + 1));
        }
    }
    std::array<double, R> rep_likelihoods;
    for( int i = 0; i < R; i++ ){
        std::vector<double> rep_likelihood;
        for( int j = 1; j < N; j++ ){
            if( m_rep_ids[j] == i )
                rep_likelihood.push_back(std::beta(m_alphas[j], m_betas[j]));
        }
        //XXX Take into account the branching factor of each rep.
        rep_likelihoods[i] = panini::maths::mean(rep_likelihood);
    }
    return argmax(rep_likelihoods)
}

template <int N, int R, typename SP>
int MRMHAPlannerDTS<N, R, SP>::replan(
        std::vector<int>* _solution,
        ReplanParams _params,
        int* _soltn_cost ){
    if (!this->check_params(_params)) { // errors printed within
        SMPL_WARN(" -> Parameters invalid");
        return 0;
    }

    m_params = _params;

    SMPL_DEBUG("Generic Search parameters:");
    SMPL_DEBUG("  Initial Epsilon: %0.3f", m_params.initial_eps);
    SMPL_DEBUG("  Final Epsilon: %0.3f", m_params.final_eps);
    SMPL_DEBUG("  Delta Epsilon: %0.3f", m_params.dec_eps);
    SMPL_DEBUG("  Return First Solution: %s", m_params.return_first_solution ? "true" : "false");
    SMPL_DEBUG("  Max Time: %0.3f", m_params.max_time);
    SMPL_DEBUG("  Repair Time: %0.3f", m_params.repair_time);
    SMPL_DEBUG("MHA Search parameters:");
    SMPL_DEBUG("  Max Expansions: %d", this->m_max_expansions);

    this->environment_->EnsureHeuristicsUpdated(true); // TODO: support backwards search

    // TODO: pick up from where last search left off and detect lazy
    // reinitializations
    reinit_search();

    this->m_eps = m_params.initial_eps;
    this->m_eps_satisfied = (double)INFINITECOST;

    // reset time limits
    this->m_num_expansions = 0;
    this->m_time_elapsed = 0.0;

    auto start_time = smpl::clock::now();

    ++this->m_call_number;
    reinit_state(m_goal_state);
    reinit_state(m_start_state);
    m_start_state->g = 0;

    for (int hidx = 0; hidx < num_heuristics(); ++hidx) {
        m_start_state->od[hidx].f = compute_key(m_start_state, hidx);
        m_open[hidx].push(&m_start_state->od[hidx]);

        m_prev_best_h[i] = m_start_state->od[hidx].h;
    }

    //reinitSearch();

    auto end_time = smpl::clock::now();
    m_time_elapsed += smpl::to_seconds(end_time - start_time);

    while (!m_open[0].empty() && !this->time_limit_reached()) {
        auto start_time = smpl::clock::now();

        // Picks a queue among all non-empty inadmissible queue.
        // If an inadmissible queue is empty, it is skipped.
        //int hidx = chooseQueue();
        int hidxrep_id = chooseRep();
        ROS_DEBUG_NAMED(LOG, "Expanding queue %d", hidx);
        ROS_DEBUG_NAMED(LOG, "==================");
        if ( get_minf(m_open[hidx]) <= m_eps_mha * get_minf(m_open[0]) ){
            if (m_goal_state->g <= get_minf(m_open[hidx])) {
                // Solution found
                this->m_eps_satisfied = m_eps*m_eps_mha;
                this->extract_path(_solution, _soltn_cost);
                return 1;
            } else {
                // Inadmissible expansion
                ROS_DEBUG_NAMED(LOG, "Inadmissible expansion");
                MRMHASearchState* s = this->state_from_open_state(m_open[hidx].min());
                ROS_DEBUG_NAMED(LOG, "State: %d, f: %d", s->state_id, s->od[hidx].f);
                this->expand(s, hidx);
                // Use the dependency matrix.
                // to partially close the state.
                for(int i = 0; i < R; i++){
                    if(this->m_rep_dependency_matrix[this->m_rep_ids[hidx]][i])
                        s->closed_in_adds[i] = true;
                }
            }
        } else {
            if (m_goal_state->g <= get_minf(m_open[0])) {
                // Solution found
                this->m_eps_satisfied = m_eps*m_eps_mha;
                this->extract_path(_solution, _soltn_cost);
                return 1;
            } else {
                //Anchor expansion
                ROS_DEBUG_NAMED(LOG, "Anchor Expansion");
                MRMHASearchState* s = this->state_from_open_state(m_open[0].min());
                this->expand(s, 0);
                s->closed_in_anc = true;
            }
        }

        auto end_time = smpl::clock::now();
        this->m_time_elapsed += smpl::to_seconds(end_time - start_time);
    }

    if (m_open[0].empty()) {
        SMPL_INFO("Anchor search exhausted");
    }
    if (time_limit_reached()) {
        SMPL_INFO("Time limit reached");
    }

    return 0;
}

#endif
