#ifndef WALKER_MHAPLNANNER_META_MHA_IMPLEMENTATION_H
#define WALKER_MHAPLNANNER_META_MHA_IMPLEMENTATION_H

#include "../mrmhaplanner_meta_mha.h"
#include <panini/maths.h>
#include <panini/algo.h>

#define LOG "mrmha_meta_mha"
#define E_LOG "mrmha_meta_mha_expansions"

template <int N, int R, typename SP>
MRMHAPlannerMetaMHA<N, R, SP>::MRMHAPlannerMetaMHA(
        DiscreteSpaceInformation* _env,
        std::array<Heuristic*, N>& _heurs,
        std::array<int, N>& _rep_ids,
        std::array<std::array<int, R>, R>& _rep_dependency_matrix,
        SP* _scheduling_policy ) :
    MRMHAPlanner<N, R, SP>( _env, _heurs, _rep_ids, _rep_dependency_matrix, _scheduling_policy ){

    // Prior
    for(int i = 0; i < R; i++)
    {
        m_alphas[i] = 1.0;
        m_betas[i] = 1.0;
    }
    for(int i = 0; i < N; i++)
    {
        m_best_h[i] = INFINITECOST;
        if(i > 0)
            m_rep_h_count[ m_rep_ids[i] ]++;
    }
    gsl_rng_env_setup();
    m_gsl_rand_T = gsl_rng_default;
    m_gsl_rand = gsl_rng_alloc(m_gsl_rand_T);

    m_delta_h_file.open("delta_h.txt", std::ios::app);
}

template <int N, int R, typename SP>
MRMHAPlannerMetaMHA<N, R, SP>::~MRMHAPlannerMetaMHA()
{
    gsl_rng_free(m_gsl_rand);
    m_delta_h_file.close();
}

template <int N, int R, typename SP>
int MRMHAPlannerMetaMHA<N, R, SP>::replan(
        double _allocated_time_sec,
        std::vector<int>* _solution ){
    int solcost;
    return replan( _allocated_time_sec, _solution, &solcost );
}

template <int N, int R, typename SP>
int MRMHAPlannerMetaMHA<N, R, SP>::replan(
        double _allocated_time_sec,
        std::vector<int>* _solution,
        int* _soltn_cost ){
    ReplanParams params = m_params;
    params.max_time = _allocated_time_sec;
    return replan( _solution, params, _soltn_cost);
}

template <int N, int R, typename SP>
int MRMHAPlannerMetaMHA<N, R, SP>::replan(
        std::vector<int>* _solution,
        ReplanParams _params,
        int* _soltn_cost ){
    if (!this->check_params(_params)) { // errors printed within
        SMPL_WARN(" -> Parameters invalid");
        return 0;
    }

    // Set heuristic offsets.
    for(int i = 1; i < num_heuristics(); i++)
        // Offset vector stores the offset = 0 for anchor too
        m_heuristic_offsets[i] = dynamic_cast<smpl::RobotHeuristic*>(this->m_h_inads[i-1])->offset;

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
    this->m_inad_expansions = 0;
    this->m_time_elapsed = 0.0;

    auto start_time = smpl::clock::now();

    ++this->m_call_number;
    reinit_state(m_goal_state);
    reinit_state(m_start_state);
    m_start_state->g = 0;

    std::vector<int> start_h, start_offsets;
    for (int hidx = 0; hidx < num_heuristics(); ++hidx)
    {
        m_start_state->od[hidx].f = compute_key(m_start_state, hidx);
        m_open[hidx].push(&m_start_state->od[hidx]);
        m_best_h[hidx] = m_start_state->od[hidx].h;
        m_initial_h[hidx] = m_start_state->od[hidx].h;
        if(hidx != 0)
        {
            start_h.push_back(m_best_h[hidx]);
            start_offsets.push_back(m_heuristic_offsets[hidx]);
        }
    }
    this->m_scheduling_policy->initialize(start_h, start_offsets);

    //reinitSearch();

    auto end_time = smpl::clock::now();
    this->m_time_elapsed += smpl::to_seconds(end_time - start_time);

    while (!m_open[0].empty() && !this->time_limit_reached()) {
        auto start_time = smpl::clock::now();

        // Picks a queue among all non-empty inadmissible queue.
        // If an inadmissible queue is empty, it is skipped.
        //int hidx = chooseQueue();
        //int rep_id = this->m_scheduling_policy->getAction();
        int hidx = this->m_scheduling_policy->getAction() + 1;
        int rep_id = m_rep_ids[hidx];
        //ROS_DEBUG_NAMED(LOG, "Expanding Rep %d", rep_id);
        //ROS_DEBUG_NAMED(LOG, "==================");
        if(m_open[hidx].empty())
            hidx = 0;
        ROS_DEBUG_NAMED(LOG, "  Size of hidx %d: %d", hidx, m_open[hidx].size());
        if ( get_minf(m_open[hidx]) <= m_eps_mha * get_minf(m_open[0]) ){
            if (m_goal_state->g <= get_minf(m_open[hidx])) {
                // Solution found
                this->m_eps_satisfied = m_eps*m_eps_mha;
                this->extract_path(_solution, _soltn_cost);
                return 1;
            } else {
                // Inadmissible expansion
                ROS_DEBUG_NAMED(LOG, "Inadmissible expansion");
                auto s = this->state_from_open_state(m_open[hidx].min());
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
                auto s = this->state_from_open_state(m_open[0].min());
                this->expand(s, 0);
                s->closed_in_anc = true;
            }
        }

        // Policy doesn't know about anchor
        for(int j = 1; j < num_heuristics(); j++)
        {
            //m_heuristic_offsets[j] = dynamic_cast<smpl::RobotHeuristic*>(this->m_h_inads[j-1])->offset;
            this->m_scheduling_policy->updateMinH(j - 1, m_best_h[j], m_heuristic_offsets[j]);
            ROS_DEBUG_NAMED(LOG, "  hidx: %d, best_h: %d, offset: %d", j, m_best_h[j], m_heuristic_offsets[j]);
        }

        this->m_scheduling_policy->updatePolicy(hidx - 1, m_best_h[hidx], m_heuristic_offsets[hidx]);
        auto end_time = smpl::clock::now();
        this->m_time_elapsed += smpl::to_seconds(end_time - start_time);
    }

    if (m_open[0].empty()) {
        SMPL_INFO("Anchor search exhausted");
    }
    if (this->time_limit_reached()) {
        SMPL_INFO("Time limit reached");
    }

    return 0;
}

template <int N, int R, typename SP>
void MRMHAPlannerMetaMHA<N, R, SP>::expand(MRMHASearchState* _state, int _hidx)
{
    m_iter++;
    int rep_id = m_rep_ids[_hidx];
    // Inserts _state into the closed queues determined by the
    // rep_dependency_matrix.
    ROS_DEBUG_NAMED(LOG, "Expanding state %d in search %d", _state->state_id, _hidx);

    assert(!this->closed_in_add_search(_state, rep_id) || !this->closed_in_anc_search(_state));

    ++this->m_num_expansions;
    if(_hidx != 0)
        ++this->m_inad_expansions;

    // remove s from  all OPEN lists.
    for (int hidx = 0; hidx < num_heuristics(); ++hidx) {
        if (m_open[hidx].contains(&_state->od[hidx])) {
            m_open[hidx].erase(&_state->od[hidx]);
        }
    }

    std::vector<int> succ_ids;
    std::vector<int> costs;
    this->environment_->GetSuccs(_state->state_id, rep_id, &succ_ids, &costs);
    assert(succ_ids.size() == costs.size());

    for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)  {
        const int cost = costs[sidx];
        MRMHASearchState* succ_state = this->get_state(succ_ids[sidx]);
        reinit_state(succ_state);

        int new_g = _state->g + costs[sidx];
        if (new_g < succ_state->g) {
            succ_state->g = new_g;
            succ_state->bp = _state;
            if (!this->closed_in_anc_search(succ_state)) {
                succ_state->od[0].f = compute_key(succ_state, 0);
                this->insert_or_update(succ_state, 0);
                //ROS_DEBUG_NAMED(LOG, "Inserted/Updated successor %d in anchor", succ_state->state_id);

                // unless it's been closed in an inadmissible search...
                //if (closed_in_add_search(succ_state, rep_id)) {
                //    continue;
                //}

                // insert into the OPEN list for each heuristic
                // XXX Currently does full sharing of all states.
                // The possibility of having a learnt ``sharing matrix`` is
                // open.
                int prev_best_h = m_best_h[_hidx];
                for (int hidx = 1; hidx < num_heuristics(); ++hidx) {
                    int fi = compute_key(succ_state, hidx);
                    // if write_stats
                    //int prev_best_h = m_best_h[hidx];

                    if(!this->closed_in_add_search(succ_state, m_rep_ids[hidx])){
                        if(fi <= m_eps_mha * succ_state->od[0].f){
                            succ_state->od[hidx].f = fi;
                            this->insert_or_update(succ_state, hidx);

                            // Meta A
                            if(succ_state->od[hidx].h < m_best_h[hidx])
                            {
                                ROS_DEBUG_NAMED(E_LOG, "  Updated best h: %d", m_best_h[hidx]);
                                m_best_h[hidx] = succ_state->od[hidx].h;
                            }
                            //ROS_DEBUG_NAMED(E_LOG, "Inserted/Updated successor %d in queue %d", succ_state->state_id, hidx);
                        }
                    }
                    //if(m_write_stats)
                        //m_delta_h_file<< hidx<< " "<< prev_best_h - m_best_h[hidx]<< "\n";
                }
                //if(m_write_stats)
                    //m_delta_h_file<< _hidx<< " "<< m_iter<< " "<<  prev_best_h - m_best_h[_hidx]<< "\n";
            }
        }
    }
    //ros::Duration(0.1).sleep();
}

#endif
