#ifndef WALKER_MHAPLNANNER_CONTEXTUAL_BANDITS_IMPLEMENTATION_H
#define WALKER_MHAPLNANNER_CONTEXTUAL_BANDITS_IMPLEMENTATION_H

#include "../mrmhaplanner_cobandits.h"
#include <panini/maths.h>
#include <panini/algo.h>

#define LOG "mrmha_cobandits"

template <int N, int R, typename SP, typename C>
MRMHAPlannerCoBandits<N, R, SP, C>::MRMHAPlannerCoBandits(
        DiscreteSpaceInformation* _env,
        std::array<Heuristic*, N>& _heurs,
        std::array<int, N>& _rep_ids,
        std::array<std::array<int, R>, R>& _rep_dependency_matrix,
        SP* _scheduling_policy,
        C* _context) :
    MRMHAPlanner<N, R, SP>( _env, _heurs, _rep_ids, _rep_dependency_matrix, _scheduling_policy ),
    m_context{_context}
{
    // Prior
    for(int i = 0; i < R; i++)
    {
        m_alphas[i] = 1.0;
        m_betas[i] = 1.0;
    }
    for(int i = 0; i < N; i++)
    {
        m_prev_best_h[i] = INFINITECOST;
        if(i > 0)
            m_rep_h_count[ m_rep_ids[i] ]++;
    }
  gsl_rng_env_setup();
  m_gsl_rand_T = gsl_rng_default;
  m_gsl_rand = gsl_rng_alloc(m_gsl_rand_T);
}

template <int N, int R, typename SP, typename C>
MRMHAPlannerCoBandits<N, R, SP, C>::~MRMHAPlannerCoBandits()
{
    gsl_rng_free(m_gsl_rand);
}

template <int N, int R, typename SP, typename C>
int MRMHAPlannerCoBandits<N, R, SP, C>::replan(
        double _allocated_time_sec,
        std::vector<int>* _solution ){
    int solcost;
    return replan( _allocated_time_sec, _solution, &solcost );
}

template <int N, int R, typename SP, typename C>
int MRMHAPlannerCoBandits<N, R, SP, C>::replan(
        double _allocated_time_sec,
        std::vector<int>* _solution,
        int* _soltn_cost ){
    ReplanParams params = m_params;
    params.max_time = _allocated_time_sec;
    return replan( _solution, params, _soltn_cost);
}

template <int N, int R, typename SP, typename C>
int MRMHAPlannerCoBandits<N, R, SP, C>::replan(
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

    for (int hidx = 0; hidx < num_heuristics(); ++hidx)
    {
        m_start_state->od[hidx].f = compute_key(m_start_state, hidx);
        m_open[hidx].push(&m_start_state->od[hidx]);
        m_prev_best_h[hidx] = m_start_state->od[hidx].h;
    }

    //reinitSearch();

    auto end_time = smpl::clock::now();
    this->m_time_elapsed += smpl::to_seconds(end_time - start_time);

    while (!m_open[0].empty() && !this->time_limit_reached()) {
        auto start_time = smpl::clock::now();

        std::vector<decltype(m_context->getContext(1))> contexts;
        //std::vector<C::ContextArray> contexts;
        std::vector<int> rep_ids;

        auto features_start_time = smpl::clock::now();
        for( int hidx = 1; hidx < num_heuristics(); hidx++ )
        {
            // Dummy state_id referring to the start.
            // Always valid.
            int state_id = 1;
            // Pass a negative rep_id if inadmissible queue is empty.
            int rep_id = -1;
            if(!m_open[hidx].empty())
            {
                if ( get_minf(m_open[hidx]) <= m_eps_mha * get_minf(m_open[0]) ){
                    if (m_goal_state->g <= get_minf(m_open[hidx])) {
                        // Solution found
                        this->m_eps_satisfied = m_eps*m_eps_mha;
                        this->extract_path(_solution, _soltn_cost);

                        ROS_WARN("Planning Stats:");
                        ROS_INFO("Total time: %f", this->m_stats.total);
                        ROS_INFO("Expand time: %f", this->m_stats.expand);
                        ROS_INFO("Features time: %f", this->m_stats.features);
                        ROS_INFO("Scheduling time: %f", this->m_stats.scheduling);
                        return 1;
                    }
                } else {
                    if (m_goal_state->g <= get_minf(m_open[0])) {
                        // Solution found
                        this->m_eps_satisfied = m_eps*m_eps_mha;
                        this->extract_path(_solution, _soltn_cost);

                        ROS_WARN("Planning Stats:");
                        ROS_INFO("Total time: %f", this->m_stats.total);
                        ROS_INFO("Expand time: %f", this->m_stats.expand);
                        ROS_INFO("Features time: %f", this->m_stats.features);
                        ROS_INFO("Scheduling time: %f", this->m_stats.scheduling);
                        return 1;
                    }
                }

                state_id = this->state_from_open_state(m_open[hidx].min())->state_id;
                rep_id = m_rep_ids[hidx];
            }
            // Context generator class should handle the conversion from
            // state-id to the actual robot state.
            // The planner is agnostic to the robot state.
            auto context = m_context->getContext(state_id);
            contexts.push_back(context);
            rep_ids.push_back(rep_id);
        }
        auto features_end_time = smpl::clock::now();
        this->m_stats.features +=  smpl::to_seconds(features_end_time - features_start_time) / 1000;
        auto scheduling_start_time = smpl::clock::now();

        int hidx = this->m_scheduling_policy->getArm(contexts, rep_ids) + 1; // Policy doesn't know about anchor
        int rep_id = m_rep_ids[hidx];
        auto context = contexts[hidx-1];
        ROS_DEBUG_NAMED(LOG, "Expanding Queue %d in Rep: %d", hidx, rep_id);
        ROS_DEBUG_NAMED(LOG, "==================");
        int reward = 0;
        if ( get_minf(m_open[hidx]) <= m_eps_mha * get_minf(m_open[0]) ){
            // Inadmissible expansion
            ROS_DEBUG_NAMED(LOG, "Inadmissible expansion");
            auto s = this->state_from_open_state(m_open[hidx].min());
            ROS_DEBUG_NAMED(LOG, "State: %d, f: %d", s->state_id, s->od[hidx].f);
            auto expand_start_time = smpl::clock::now();

            this->expand(s, hidx);

            auto expand_end_time = smpl::clock::now();
            this->m_stats.expand +=  smpl::to_seconds(expand_end_time - expand_start_time);
            // Use the dependency matrix.
            // to partially close the state.
            for(int i = 0; i < R; i++){
                if(this->m_rep_dependency_matrix[this->m_rep_ids[hidx]][i])
                    s->closed_in_adds[i] = true;
            }
            // DTS
            if( m_best_h[hidx] < m_prev_best_h[hidx] )
            {
                m_prev_best_h[hidx] = m_best_h[hidx];
                reward++;
            }
        } else {
            //Anchor expansion
            ROS_DEBUG_NAMED(LOG, "Anchor Expansion");
            auto s = this->state_from_open_state(m_open[0].min());

            auto expand_start_time = smpl::clock::now();

            this->expand(s, 0);

            auto expand_end_time = smpl::clock::now();
            this->m_stats.expand +=  smpl::to_seconds(expand_end_time - expand_start_time);
            s->closed_in_anc = true;
        }
        ROS_DEBUG_NAMED(LOG, "Size of Anchor: %d", m_open[0].size());
        scheduling_start_time = smpl::clock::now();

        this->m_scheduling_policy->updatePolicy(context, reward, rep_id);

        auto end_time = smpl::clock::now();
        this->m_stats.scheduling += smpl::to_seconds(end_time - scheduling_start_time);
        this->m_time_elapsed += smpl::to_seconds(end_time - start_time);
    }
    this->m_stats.total = this->m_time_elapsed;

    if (m_open[0].empty()) {
        SMPL_INFO("Anchor search exhausted");
    }
    if (this->time_limit_reached()) {
        SMPL_INFO("Time limit reached");
    }
    ROS_WARN("Planning Stats:");
    ROS_INFO("Total time: %f", this->m_stats.total);
    ROS_INFO("Expand time: %f", this->m_stats.expand);
    ROS_INFO("Features time: %f", this->m_stats.features);
    ROS_INFO("Scheduling time: %f", this->m_stats.scheduling);

    return 0;
}

template <int N, int R, typename SP, typename C>
void MRMHAPlannerCoBandits<N, R, SP, C>::expand(MRMHASearchState* _state, int _hidx)
{
    int rep_id = m_rep_ids[_hidx];
    // Inserts _state into the closed queues determined by the
    // rep_dependency_matrix.
    ROS_DEBUG_NAMED(LOG, "Expanding state %d in search %d", _state->state_id, _hidx);

    assert(!this->closed_in_add_search(_state, rep_id) || !this->closed_in_anc_search(_state));

    ++this->m_num_expansions;

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
                for (int hidx = 1; hidx < num_heuristics(); ++hidx) {
                    int fi = compute_key(succ_state, hidx);
                    if(!this->closed_in_add_search(succ_state, m_rep_ids[hidx])){
                        //if(fi <= m_eps_mha * succ_state->od[0].f){
                            succ_state->od[hidx].f = fi;
                            this->insert_or_update(succ_state, hidx);
                            if(succ_state->od[hidx].h < m_prev_best_h[hidx])
                                m_best_h[hidx] = succ_state->od[hidx].h;
                            ROS_DEBUG_NAMED(LOG, "Inserted/Updated successor %d in Queue %d", succ_state->state_id, hidx);
                        //}
                    }
                }
            }
        }
    }
    //ros::Duration(0.1).sleep();
}

#endif
