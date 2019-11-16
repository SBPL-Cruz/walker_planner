#ifndef WALKER_MRMHAPLANNER_DETAIL_H
#define WALKER_MRMHAPLANNER_DETAIL_H

#include <limits>
#include <smpl/time.h>
#include "../mrmhaplanner.h"

#define LOG "templated_mrmha"

template <int N, int R, typename SP>
MRMHAPlanner<N, R, SP>::MRMHAPlanner(
        DiscreteSpaceInformation* _env,
        std::array<Heuristic*, N>& _heurs,
        std::array<int, N>& _rep_ids,
        std::array<std::array<int, R>, R>& _rep_dependency_matrix,
        SP* _scheduling_policy) :
    SBPLPlanner(),
    m_h_anchor{_heurs[0]},
    m_h_inads{_heurs.data() + 1},
    m_h_count{N},
    m_scheduling_policy{_scheduling_policy},
    m_rep_ids{_rep_ids},
    m_rep_dependency_matrix{_rep_dependency_matrix},
    m_params{0.0},
    m_eps{1.0},
    m_eps_mha{1.0},
    m_num_expansions{0},
    m_time_elapsed{0.0},
    m_start_state{NULL},
    m_goal_state{NULL},
    m_open{NULL} {

    this->environment_ = _env;
    m_open = new CHeap[m_h_count];

    m_params.initial_eps = 1.0;
    m_params.final_eps = 1.0;
    m_params.dec_eps = 0.0; // NOTE: same initial epsilon delta as ARA*
    m_params.return_first_solution = false;
    m_params.max_time = 0.0;
    m_params.repair_time = 0.0;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    m_generator.seed(seed);
}

template <int N, int R, typename SP>
int MRMHAPlanner<N, R, SP>::replan(
        double _allocated_time_sec,
        std::vector<int>* _solution ){
    int solcost;
    return replan( _allocated_time_sec, _solution, &solcost );
}

template <int N, int R, typename SP>
int MRMHAPlanner<N, R, SP>::replan(
        double _allocated_time_sec,
        std::vector<int>* _solution,
        int* _soltn_cost ){
    ReplanParams params = m_params;
    params.max_time = _allocated_time_sec;
    return replan( _solution, params, _soltn_cost);
}

template <int N, int R, typename SP>
int MRMHAPlanner<N, R, SP>::replan(
        std::vector<int>* _solution,
        ReplanParams _params,
        int* _soltn_cost ){
    if (!check_params(_params)) { // errors printed within
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
    SMPL_DEBUG("  Max Expansions: %d", m_max_expansions);

    environment_->EnsureHeuristicsUpdated(true); // TODO: support backwards search

    // TODO: pick up from where last search left off and detect lazy
    // reinitializations
    reinit_search();

    m_eps = m_params.initial_eps;
    m_eps_satisfied = (double)INFINITECOST;

    // reset time limits
    m_num_expansions = 0;
    m_time_elapsed = 0.0;

    auto start_time = smpl::clock::now();

    ++m_call_number;
    reinit_state(m_goal_state);
    reinit_state(m_start_state);
    m_start_state->g = 0;

    for (int hidx = 0; hidx < num_heuristics(); ++hidx) {
        m_start_state->od[hidx].f = compute_key(m_start_state, hidx);
        m_open[hidx].push(&m_start_state->od[hidx]);
    }

    //reinitSearch();

    auto end_time = smpl::clock::now();
    m_time_elapsed += smpl::to_seconds(end_time - start_time);

    while (!m_open[0].empty() && !time_limit_reached()) {
        auto start_time = smpl::clock::now();

        // Picks a queue among all non-empty inadmissible queue.
        // If an inadmissible queue is empty, it is skipped.
        int hidx = chooseQueue();
        ROS_DEBUG_NAMED(LOG, "Expanding queue %d", hidx);
        ROS_DEBUG_NAMED(LOG, "==================");
        if ( get_minf(m_open[hidx]) <= m_eps_mha * get_minf(m_open[0]) ){
            if (m_goal_state->g <= get_minf(m_open[hidx])) {
                // Solution found
                m_eps_satisfied = m_eps*m_eps_mha;
                extract_path(_solution, _soltn_cost);
                return 1;
            } else {
                // Inadmissible expansion
                ROS_DEBUG_NAMED(LOG, "Inadmissible expansion");
                MRMHASearchState* s = state_from_open_state(m_open[hidx].min());
                ROS_DEBUG_NAMED(LOG, "State: %d, f: %d", s->state_id, s->od[hidx].f);
                expand(s, hidx);
                // Use the dependency matrix.
                // to partially close the state.
                for(int i = 0; i < R; i++){
                    if(m_rep_dependency_matrix[m_rep_ids[hidx]][i])
                        s->closed_in_adds[i] = true;
                }
            }
        } else {
            if (m_goal_state->g <= get_minf(m_open[0])) {
                // Solution found
                m_eps_satisfied = m_eps*m_eps_mha;
                extract_path(_solution, _soltn_cost);
                return 1;
            } else {
                //Anchor expansion
                ROS_DEBUG_NAMED(LOG, "Anchor Expansion");
                MRMHASearchState* s = state_from_open_state(m_open[0].min());
                expand(s, 0);
                s->closed_in_anc = true;
            }
        }

        auto end_time = smpl::clock::now();
        m_time_elapsed += smpl::to_seconds(end_time - start_time);
    }

    if (m_open[0].empty()) {
        SMPL_INFO("Anchor search exhausted");
    }
    if (time_limit_reached()) {
        SMPL_INFO("Time limit reached");
    }

    return 0;
}

template <int N, int R, typename SP>
int MRMHAPlanner<N, R, SP>::set_goal(int _goal_stateID) {
    SMPL_DEBUG("Planner set goal");
    m_goal_state = get_state(_goal_stateID);
    if (!m_goal_state) {
        return 0;
    } else {
        return 1;
    }
}
template <int N, int R, typename SP>
int MRMHAPlanner<N, R, SP>::set_start(int _start_stateID) {
    m_start_state = get_state(_start_stateID);
    if (!m_start_state) {
        return 0;
    } else {
        return 1;
    }
}

template <int N, int R, typename SP>
bool MRMHAPlanner<N, R, SP>::check_params(const ReplanParams& _params){
    if (_params.initial_eps < 1.0) {
        SMPL_ERROR("Initial Epsilon must be greater than or equal to 1");
        return false;
    }

    if (_params.final_eps > _params.initial_eps) {
        SMPL_ERROR("Final Epsilon must be less than or equal to initial epsilon");
        return false;
    }

    if (_params.dec_eps < 0.0) {
        SMPL_ERROR("Delta epsilon must be positive");
        return false;
    }

    if (_params.return_first_solution &&
        _params.max_time <= 0.0 &&
        m_max_expansions <= 0)
    {
        SMPL_ERROR("Max Time or Max Expansions must be positive");
        return false;
    }

    return true;

}

template <int N, int R, typename SP>
bool MRMHAPlanner<N, R, SP>::time_limit_reached(){
    if (m_params.return_first_solution) {
        return false;
    } else if (m_params.max_time > 0.0 && m_time_elapsed >= m_params.max_time) {
        return true;
    } else if (m_max_expansions > 0 && m_num_expansions >= m_max_expansions) {
        return true;
    } else {
        return false;
    }
}

template <int N, int R, typename SP>
void MRMHAPlanner<N, R, SP>::init_state( MRMHASearchState* _state, size_t _mha_state_idx, int _state_id ){
    _state->call_number = 0; // not initialized for any iteration
    _state->state_id = _state_id;
    _state->closed_in_anc = false;
    for( auto& ele : _state->closed_in_adds )
        ele = false;
    for (int i = 0; i < num_heuristics(); ++i) {
        _state->od[i].f = compute_heuristic(_state->state_id, i);
        _state->od[i].me = _state;
    }

    //SMPL_DEBUG_STREAM("Initialized state: " << *_state);
    for (int i = 0; i < num_heuristics(); ++i) {
        SMPL_DEBUG("  me[%d]: %p", i, _state->od[i].me);
        SMPL_DEBUG("  h[%d]: %d", i, _state->od[i].h);
        SMPL_DEBUG("  f[%d]: %d", i, _state->od[i].f);
    }
}

template <int N, int R, typename SP>
void MRMHAPlanner<N, R, SP>::reinit_state(MRMHASearchState* _state){
    if (_state->call_number != m_call_number) {
        _state->call_number = m_call_number;
        _state->g = INFINITECOST;
        _state->bp = nullptr;

        _state->closed_in_anc = false;
        for(auto& ele : _state->closed_in_adds)
            ele = false;

        for (int i = 0; i < num_heuristics(); ++i) {
            _state->od[i].h = compute_heuristic(_state->state_id, i);
            _state->od[i].f = INFINITECOST;
        }

        //SMPL_DEBUG_STREAM("Reinitialized state: " << *_state);
        for (int i = 0; i < num_heuristics(); ++i) {
            SMPL_DEBUG("  me[%d]: %p", i, _state->od[i].me);
            SMPL_DEBUG("  h[%d]: %d", i, _state->od[i].h);
            SMPL_DEBUG("  f[%d]: %d", i, _state->od[i].f);
        }
    }
}

template <int N, int R, typename SP>
void MRMHAPlanner<N, R, SP>::clear_open_lists(){
    for (int i = 0; i < num_heuristics(); ++i) {
        m_open[i].clear();
    }
}

template <int N, int R, typename SP>
void MRMHAPlanner<N, R, SP>::clear(){
    clear_open_lists();

    for(int i = 0; i < m_search_states.size(); i++){
        const int state_id = m_search_states[i]->state_id;
        int* idxs = environment_->StateID2IndexMapping[state_id];
        idxs[MHAMDP_STATEID2IND] = -1;
        delete m_search_states[i];
    }

    m_search_states.clear();

    m_start_state = nullptr;
    m_goal_state = nullptr;
}

template <int N, int R, typename SP>
int MRMHAPlanner<N, R, SP>::compute_key( MRMHASearchState* _state, int _hidx ){
    return _state->g + (int)(m_eps * (double)_state->od[_hidx].h);
}

template <int N, int R, typename SP>
int MRMHAPlanner<N, R, SP>::sampleIndex(const std::array<double, N>& _likelihoods){
    std::array<int, N> counts;
    // The anchor is supposed to have 0 likelihood.
    for(int i = 0; i < _likelihoods.size(); i++)
        counts[i] = 100*_likelihoods[i];
    std::discrete_distribution<int> dist(counts.begin(), counts.end());
    return dist(m_generator);
}

template <int N, int R, typename SP>
int MRMHAPlanner<N, R, SP>::chooseQueue(){
    std::array<double, N> queue_probs;
    // Do not expand the anchor unless the algorithm requires so.
    queue_probs[0] = 0.0;

    for(int hidx = 1; hidx < num_heuristics(); ++hidx){
        //Only makes sense if the queue is not empty.
        if (!m_open[hidx].empty()){
            double p = m_scheduling_policy->getActionSpaceProb(
                    state_from_open_state(m_open[hidx].min())->state_id,
                        hidx );
            queue_probs[hidx] = p;
        } else {
            queue_probs[hidx] = 0.0;
        }
    }
    return sampleIndex(queue_probs);
}

template <int N, int R, typename SP>
int MRMHAPlanner<N, R, SP>::compute_heuristic( int _state_id, int _hidx ){
    if (_hidx == 0) {
        return m_h_anchor->GetGoalHeuristic(_state_id);
    } else {
        return m_h_inads[_hidx - 1]->GetGoalHeuristic(_state_id);
    }
}

template <int N, int R, typename SP>
void MRMHAPlanner<N, R, SP>::expand(MRMHASearchState* _state, int _hidx){
    int rep_id = m_rep_ids[_hidx];
    // Inserts _state into the closed queues determined by the
    // rep_dependency_matrix.
    ROS_DEBUG_NAMED(LOG, "Expanding state %d in search %d", _state->state_id, _hidx);

    assert(!closed_in_add_search(_state, rep_id) || !closed_in_anc_search(_state));

    ++m_num_expansions;

    // remove s from  all OPEN lists.
    for (int hidx = 0; hidx < num_heuristics(); ++hidx) {
        if (m_open[hidx].contains(&_state->od[hidx])) {
            m_open[hidx].erase(&_state->od[hidx]);
        }
    }

    std::vector<int> succ_ids;
    std::vector<int> costs;
    environment_->GetSuccs(_state->state_id, rep_id, &succ_ids, &costs);
    assert(succ_ids.size() == costs.size());

    for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)  {
        const int cost = costs[sidx];
        MRMHASearchState* succ_state = get_state(succ_ids[sidx]);
        reinit_state(succ_state);

        int new_g = _state->g + costs[sidx];
        if (new_g < succ_state->g) {
            succ_state->g = new_g;
            succ_state->bp = _state;
            if (!closed_in_anc_search(succ_state)) {
                succ_state->od[0].f = compute_key(succ_state, 0);
                insert_or_update(succ_state, 0);
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
                    if(!closed_in_add_search(succ_state, m_rep_ids[hidx])){
                        if(fi <= m_eps_mha * succ_state->od[0].f){
                            succ_state->od[hidx].f = fi;
                            insert_or_update(succ_state, hidx);
                            //ROS_DEBUG_NAMED(LOG, "Inserted/Updated successor %d in Queue %d", succ_state->state_id, hidx);
                        }
                    }
                }
            }
        }
    }
    //ros::Duration(0.1).sleep();
}

template <int N, int R, typename SP>
void MRMHAPlanner<N, R, SP>::insert_or_update( MRMHASearchState* _state, int _hidx ) {
    if (m_open[_hidx].contains(&_state->od[_hidx])) {
        m_open[_hidx].update(&_state->od[_hidx]);
    } else {
        m_open[_hidx].push(&_state->od[_hidx]);
    }
}

template <int N, int R, typename SP>
void MRMHAPlanner<N, R, SP>::extract_path(std::vector<int>* _solution_path, int* _solcost){
    ROS_DEBUG("Extracting path");
    _solution_path->clear();
    *_solcost = 0;
    for (MRMHASearchState* state = m_goal_state; state; state = state->bp) {
        _solution_path->push_back(state->state_id);
        if (state->bp) {
            *_solcost += (state->g - state->bp->g);
        }
    }

    // TODO: special cases for backward search
    std::reverse(begin(*_solution_path), end(*_solution_path));
}

#undef LOG

#endif
