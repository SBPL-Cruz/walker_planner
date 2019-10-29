
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <chrono>

#include <sbpl/utils/key.h>
#include "planners/mrmhaplanner.h"

static double GetTime()
{
    return (double)clock() / (double)CLOCKS_PER_SEC;
}

MRMHAPlanner::MRMHAPlanner(
    DiscreteSpaceInformation* environment,
    Heuristic* hanchor,
    Heuristic** heurs,
    int hcount)
:
    MHAPlanner( environment, hanchor, heurs, hcount ) {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

        std::default_random_engine m_generator(seed);
    }

MRMHAPlanner::~MRMHAPlanner()
{
}

int MRMHAPlanner::replan(
        double allocated_time_sec,
        std::vector<int>* solution_stateIDs_V){
    int solcost;
    return replan(allocated_time_sec, solution_stateIDs_V, &solcost);
}

int MRMHAPlanner::replan(
        double allocated_time_sec,
        std::vector<int>* solution_stateIDs_V,
        int* solcost){
    ReplanParams params = m_params;
    params.max_time = allocated_time_sec;
    return replan(solution_stateIDs_V, params, solcost);
}

int argmax(std::vector<double>& vec){
    int max_id = 0;
    for(int i=0; i<vec.size(); i++)
        if(vec[i] > vec[max_id])
            max_id = i;
    return max_id;
}

int MRMHAPlanner::replan(
    std::vector<int>* solution_stateIDs_V,
    ReplanParams params,
    int* solcost)
{
    if (!check_params(params)) { // errors printed within
        return 0;
    }

    m_params = params;

    SBPL_INFO("Generic Search parameters:");
    SBPL_INFO("  Initial Epsilon: %0.3f", m_params.initial_eps);
    SBPL_INFO("  Final Epsilon: %0.3f", m_params.final_eps);
    SBPL_INFO("  Delta Epsilon: %0.3f", m_params.dec_eps);
    SBPL_INFO("  Return First Solution: %s", m_params.return_first_solution ? "true" : "false");
    SBPL_INFO("  Max Time: %0.3f", m_params.max_time);
    SBPL_INFO("  Repair Time: %0.3f", m_params.repair_time);
    SBPL_INFO("MHA Search parameters:");
    SBPL_INFO("  MHA Epsilon: %0.3f", m_initial_eps_mha);
    SBPL_INFO("  Max Expansions: %d", m_max_expansions);

    environment_->EnsureHeuristicsUpdated(true); // TODO: support backwards search

    // TODO: pick up from where last search left off and detect lazy
    // reinitializations
    reinit_search();

    m_eps = m_params.initial_eps;
    m_eps_mha = m_initial_eps_mha;
    m_eps_satisfied = (double)INFINITECOST;

    // reset time limits
    m_num_expansions = 0;
    m_elapsed = 0.0;

    double start_time, end_time;

    start_time = GetTime();

    ++m_call_number;
    reinit_state(m_goal_state);
    reinit_state(m_start_state);
    m_start_state->g = 0;

    // insert start state into all heaps with key(start, i) as priority
    for (int hidx = 0; hidx < num_heuristics(); ++hidx) {
        CKey key;
        key.key[0] = compute_key(m_start_state, hidx);
        m_open[hidx].insertheap(&m_start_state->od[hidx].open_state, key);
        SBPL_DEBUG("Inserted start state %d into search %d with f = %d", m_start_state->state_id, hidx, key.key[0]);
    }

    end_time = GetTime();
    m_elapsed += (end_time - start_time);

    while (!m_open[0].emptyheap() && !time_limit_reached()) {
        start_time = GetTime();
        // special case for mha* without additional heuristics
        if (num_heuristics() == 1) {
            if (m_goal_state->g <= get_minf(m_open[0])) {
                m_eps_satisfied = m_eps * m_eps_mha;
                extract_path(solution_stateIDs_V, solcost);
                return 1;
            }
            else {
                MHASearchState* s = state_from_open_state(m_open[0].getminheap());
                expand(s, 0);
            }
        }

        // Round-robin
        for( int hidx=1; hidx < num_heuristics(); ++hidx ){
            if (m_open[0].emptyheap()) {
                break;
            }

            if (!m_open[hidx].emptyheap() && get_minf(m_open[hidx]) <=
                m_eps_mha * get_minf(m_open[0])){
                auto state = state_from_open_state(m_open[hidx].getminheap());
                if (m_goal_state->g <= get_minf(m_open[hidx])) {
                    m_eps_satisfied = m_eps * m_eps_mha;
                    extract_path(solution_stateIDs_V, solcost);
                    return 1;
                }
                else {
                    MHASearchState* s =
                            state_from_open_state(m_open[hidx].getminheap());
                    expand(s, hidx);
                }
            }
            else {
                if (m_goal_state->g <= get_minf(m_open[0])) {
                    m_eps_satisfied = m_eps * m_eps_mha;
                    extract_path(solution_stateIDs_V, solcost);
                    return 1;
                }
                else {
                    MHASearchState* s =
                            state_from_open_state(m_open[0].getminheap());
                    expand(s, 0);
                }
            }
            end_time = GetTime();
            m_elapsed += (end_time - start_time);
        }
    }

    if (m_open[0].emptyheap()) {
        SBPL_DEBUG("Anchor search exhausted");
    }
    if (time_limit_reached()) {
        SBPL_DEBUG("Time limit reached");
    }

    return 0;
}

void MRMHAPlanner::expand(MHASearchState* state, int hidx)
{
    SBPL_DEBUG("Expanding state %d in search %d", state->state_id, hidx);

    assert(!closed_in_add_search(state) || !closed_in_anc_search(state));

    if (hidx == 0) {
        state->closed_in_anc = true;
    }
    else {
        state->closed_in_add = true;
    }
    ++m_num_expansions;

    // remove s from all open lists
    for (int temp_hidx = 0; temp_hidx < num_heuristics(); ++temp_hidx) {
        if (m_open[temp_hidx].inheap(&state->od[temp_hidx].open_state)) {
            m_open[temp_hidx].deleteheap(&state->od[temp_hidx].open_state);
        }
    }

    std::vector<int> succ_ids;
    std::vector<int> costs;
    // Call the right ActionSpace.
    //environment_->GetSuccs(state->state_id, hidx, &succ_ids, &costs);
    //XXX Test with full body rep.
    environment_->GetSuccs(state->state_id, 0, &succ_ids, &costs);
    assert(succ_ids.size() == costs.size());

    for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)  {
        const int cost = costs[sidx];
        MHASearchState* succ_state = get_state(succ_ids[sidx]);
        reinit_state(succ_state);

        SBPL_DEBUG(" Successor %d", succ_state->state_id);

        int new_g = state->g + costs[sidx];
        if (new_g < succ_state->g) {
            succ_state->g = new_g;
            succ_state->bp = state;
            if (!closed_in_anc_search(succ_state)) {
                const int fanchor = compute_key(succ_state, 0);
                insert_or_update(succ_state, 0, fanchor);
                SBPL_DEBUG("  Update in search %d with f = %d", 0, fanchor);

                if (!closed_in_add_search(succ_state)) {
                    for (int temp_hidx = 1; temp_hidx < num_heuristics(); ++temp_hidx) {
                        int fn = compute_key(succ_state, temp_hidx);
                        if (fn <= m_eps_mha * fanchor) {
                            insert_or_update(succ_state, temp_hidx, fn);
                            SBPL_DEBUG("  Update in search %d with f = %d", temp_hidx, fn);
                        }
                        else {
                            SBPL_DEBUG("  Skipping update of in search %d (%0.3f > %0.3f)", temp_hidx, (double)fn, m_eps_mha * fanchor);
                        }
                    }
                }
            }
        }
    }

    assert(closed_in_any_search(state));
}
