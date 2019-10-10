#ifndef WALKER_MRMHAPLANNER_DETAIL_H
#define WALKER_MRMHAPLANNER_DETAIL_H

#include "../mrmhaplanner.h"

template <int N, int R>
MRMHAPlanner<N, R>::MRMHAPlanner(
        DiscreteSpaceInformation* _env,
        std::array<Heuristic*, N> _heurs,
        std::arra<int, N> _rep_ids,
        std::array<std::array<int, N>, N> rep_dependency_matrix) :
    SBPLPlanner(),
    environment_{_env},
    m_h_anchor{_heurs[0]},
    m_h_inads{_heurs.data() + 1},
    m_h_count{N},
    m_rep_ids{_rep_ids},
    m_rep_dependency_matrix{_rep_dependency_matrix},
    m_params{0.0},
    m_eps{1.0},
    m_eps_mhs{1.0},
    m_num_expansions{0},
    m_time_elapsed{0.0},
    m_start_state{NULL},
    m_goal_state{NULL},
    m_open{NULL} {

    m_open = new CHeap[m_h_count];

    m_params.initial_eps = 1.0;
    m_params.final_eps = 1.0;
    m_params.dec_eps = 0.0; // NOTE: same initial epsilon delta as ARA*
    m_params.return_first_solution = false;
    m_params.max_time = 0.0;
    m_params.repair_time = 0.0;
}

int MRMHAPlanner::replan(
        double _allocated_time_sec,
        std::vector<int>* _solution ){
    int solcost;
    return replan( _allocated_time_sec, _solution, &solcost );
}

int MRMHAPlanne::replan(
        double _allocated_time_sec,
        std::vector<int>* _solution,
        int* _soltn_cost ){
    ReplanParams params = m_params;
    params.max_time = _allocated_time_sec;
    return replan( _solution, params, _solcost );
}

int MRMHAPlanne::replan(
        std::vector<int>* _solution,
        ReplanParams _params,
        int* _soltn_cost ){
    if (!check_params(params)) { // errors printed within
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
    m_elapsed = 0.0;

    auto start_time = smpl::clock::now();

    ++m_call_number;
    reinit_state(m_goal_state);
    reinit_state(m_start_state);
    m_start_state->g = 0;

    SMPL_INFO("Insert start state into OPEN and PSET");

    // insert start state into OPEN with g(s) + h(s) as the priority
    // insert start state into PSET and place in all RANK lists
    m_start_state->od[0].f = compute_key(m_start_state, 0);
    m_open[0].push(&m_start_state->od[0]);
    for (int hidx = 1; hidx < num_heuristics(); ++hidx) {
        m_start_state->od[hidx].f = compute_key(m_start_state, hidx);
        m_open[hidx].push(&m_start_state->od[hidx]);
        SMPL_INFO("Inserted start state %d into search %d with f = %d", m_start_state->state_id, hidx, m_start_state->od[hidx].f);
    }

    //reinitSearch();

    auto end_time = smpl::clock::now();
    m_elapsed += to_seconds(end_time - start_time);

    while (!m_open[0].empty() && !time_limit_reached()) {
        auto start_time = smpl::clock::now();

        // Picks a queue among all non-empty inadmissible queue.
        // If an inadmissible queue is empty, it is skipped.
        int hidx = chooseInadQueue();

        if (m_goal_state->g <= get_minf(m_open[hidx])) {
            // Solution found
            m_eps_satisfied = m_eps;
            extract_path(solution, solcost);
            return 1;
        }

        if ( get_minf(m_open[hidx]) <= m_eps_mha * get_minf(m_open[0]) ){
            // Inadmissible expansion
            MRMHASearchState<N, R>* s = state_from_open_state(m_open[hidx].min());
            expand(s, hidx);
            // Use the dependency matrix.
            // to partially close the state.
            for( auto& ele : m_rep_dependency_matrix[m_rep_ids[hidx]])
                if(ele)
                    s->closed_in_adds[m_rep_ids[hidx]] = true;
        } else {
            //Anchor expansion
            MRMHASearchState<N, R>* s = state_from_open_state(m_open[0].min());
            expand(s, 0);
            s->closed_in_anc = true;
        }

        auto end_time = smpl::clock::now();
        m_elapsed += to_seconds(end_time - start_time);
    }

    if (m_open[0].empty()) {
        SMPL_INFO("Anchor search exhausted");
    }
    if (time_limit_reached()) {
        SMPL_INFO("Time limit reached");
    }

    return 0;

}

template <int N, int R>
MRMHASearchState<N, R>* MRMHAPlanner::get_state(int _state_id){
    assert(_state_id >= 0 && _state_id < environment_->StateID2IndexMapping.size());
    int* idxs = environment_->StateID2IndexMapping[state_id];
    if (idxs[MHAMDP_STATEID2IND] == -1) {
        MRMHASearchState<N, R> s;

        const size_t mha_state_idx = m_search_states.size();
        init_state(&s, mha_state_idx, state_id);

        // map graph state to search state
        idxs[MHAMDP_STATEID2IND] = (int)mha_state_idx;

        m_search_states.push_back(std::move(s));

        return &(*m_search_states.back());
    }
    else {
        int ssidx = idxs[MHAMDP_STATEID2IND];
        return m_search_states[ssidx];
    }
}

template <int N, int R>
void MRMHAPlanner<N, R>::init_state( MRMHASearchState<N, R>* _state, size_t _mha_state_idx, int _state_id ){
    _state->call_number = 0; // not initialized for any iteration
    _state->state_id = _state_id;
    _state->closed_in_anc = false;
    for( auto& ele : _state->closed_in_adds )
        ele = false;
    for (int i = 0; i < num_heuristics(); ++i) {
        _state->od[i].f = compute_heuristic(_state->state_id, i);
        _state->od[i].me = _state;
    }

    SMPL_DEBUG_STREAM("Initialized state: " << *state);
    for (int i = 0; i < num_heuristics(); ++i) {
        SMPL_DEBUG("  me[%d]: %p", i, _state->od[i].me);
        SMPL_DEBUG("  h[%d]: %d", i, _state->od[i].h);
        SMPL_DEBUG("  f[%d]: %d", i, _state->od[i].f);
    }
}

template <int N, int R>
void MRMHAPlanner<N, R>::reinit_state(MRMHASearchState<N, R>* _state){
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

        SMPL_DEBUG_STREAM("Reinitialized state: " << *_state);
        for (int i = 0; i < num_heuristics(); ++i) {
            SMPL_DEBUG("  me[%d]: %p", i, _state->od[i].me);
            SMPL_DEBUG("  h[%d]: %d", i, _state->od[i].h);
            SMPL_DEBUG("  f[%d]: %d", i, _state->od[i].f);
        }
    }
}

template <int N, int R>
void MRMHAPlanner<N, R>::clear_open_lists(){
    for (int i = 0; i < num_heuristics(); ++i) {
        m_open[i].clear();
    }
}

template <int N, int R>
void MRMHAPlanner<N, R>::clear(){
    clear_open_lists();

    m_search_states.clear();

    m_start_state = nullptr;
    m_goal_state = nullptr;
}

template <int N, int R>
int MRMHAPlanner<N, R>::compute_key( MRMHASearchState<N, R>* _state, int _hidx ){
    return _state->g + (int)(m_eps * (double)_state->od[hidx].h);
}

template <int N, int R>
MRMHAPlanner<N, R>::compute_heuristic( int _state_id, int _hidx ){
    if (_hidx == 0) {
        return m_h_anchor->GetGoalHeuristic(_state_id);
    } else {
        return m_h_inads[_hidx - 1]->GetGoalHeuristic(_state_id);
    }
}

template <int N, int R>
MRMHAPlanner<N, R>::expand(MRMHASearchState<N, R>* _state, int _hidx){
    int rep_id = m_rep_ids[hidx];
    // Inserts _state into the closed queues determined by the
    // rep_dependency_matrix.
    SMPL_DEBUG("Expanding state %d in search %d", _state->state_id, _hidx);

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
        MRMHASearchState<N, R>* succ_state = get_state(succ_ids[sidx]);
        reinit_state(succ_state);

        int new_g = state->g + costs[sidx];
        if (new_g < succ_state->g) {
            succ_state->g = new_g;
            succ_state->bp = state;
            if (!closed_in_anc_search(succ_state)) {
                succ_state->od[0].f = compute_key(succ_state, 0);
                insert_or_update(succ_state, 0);

                // unless it's been closed in an inadmissible search...
                if (closed_in_add_search(succ_state, rep_id)) {
                    continue;
                }

                // insert into the OPEN list for each heuristic
                // XXX Currently does full sharing of all states.
                // The possibility of having a learnt ``sharing matrix`` is
                // open.
                for (int hidx = 1; hidx < num_heuristics(); ++hidx) {
                    if(!closed_in_add_search(succ_state, m_rep_ids[hidx])){
                        succ_state->od[hidx].f = compute_key(succ_state, hidx);
                        insert_or_update(succ_state, hidx);
                    }
                }
            }
        }
    }
}

template <int N, int R>
MRMHAPlanner<N, R>::insert_or_update( MRMHASearchState<N, R>* _state, int _hidx ) {
    if (m_open[_hidx].contains(&_state->od[_hidx])) {
        m_open[_hidx].update(&_state->od[_hidx]);
    } else {
        m_open[_hidx].push(&_state->od[_hidx]);
    }
}

template <int N, int R>
MRMHAPlanner<N, R>::extract_path(std::vector<int>* _solution_path, int* _solcost){
    SMPL_INFO("Extracting path");
    _solution_path->clear();
    *_solcost = 0;
    for (MRMHASearchState<N, R>* state = m_goal_state; state; state = state->bp) {
        _solution_path->push_back(state->state_id);
        if (state->bp) {
            *_solcost += (state->g - state->bp->g);
        }
    }

    // TODO: special cases for backward search
    std::reverse(begin(*_solution_path), end(*_solution_path));
}

#endif
