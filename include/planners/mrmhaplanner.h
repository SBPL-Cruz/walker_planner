#ifndef WALKER_MRMHAPLANNER_H
#define WALKER_MRMHAPLANNER_H

#include <algorithm>

#include <sbpl/planners/planner.h>
#include <sbpl/heuristics/heuristic.h>
#include <sbpl/utils/heap.h>

template <int N, int R>
struct MRMHASearchState {
    int call_number;
    int state_id;
    int g;
    MRMHASearchState<N, R>* bp;
    bool closed_in_anc;
    std::array<int, R> closed_in_adds;

    struct HeapData : public heap_element
    {
        MRMHASearchState<N, R>* me;
        int h;
        int f;
    };

    std::array<HeapData, N> od;
};

/**
 * N : Number of queues
 * R : Number of representations
 * SP: Scheduling Policy
 */
template <int N, int R, typename SP>
class MRMHAPlanner : public SBPLPlanner {
    public:
    /** Each heuristic (anchor and inadmissible) has a corresponding rep_id.
     * The rep_dependency_matrix is a max(rep_ids) x max(rep_ids) size matrix.
     * If a state is partially closed with respect to a representation i,
     * it determines the other representations with respect to which it is also
     * closed.
     * By convention, the first heuristic is the anchor heuristic and it should
     * get the union of all representations.
     */
    MRMHAPlanner(
        // Ideally, I should have a MultiRepDiscreteSpace
        DiscreteSpaceInformation* env,
        std::array<Heuristic*, N> heurs,
        std::array<int, N> rep_ids,
        std::array<std::array<int, R>, R> rep_dependency_matrix,
        SP* scheduling_policy );

    virtual ~MRMHAPlanner();

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
    virtual int set_goal(int goal_stateID) override;
    virtual int set_start(int start_stateID) override;
    virtual int force_planning_from_scratch() override { return 0; }
    virtual int force_planning_from_scratch_and_free_memory() override { return 0; }
    virtual int set_search_mode(bool bSearchUntilFirstSolution) override {  }
    virtual void costs_changed(StateChangeQuery const & stateChange) override {  }
    //}

    /// Override SBPLPlanner methods.
    //{
    virtual double get_solution_eps() const override {
        return m_eps*m_eps_mha;
    }

    virtual int get_n_expands() const override {
        return m_num_expansions;
    }

    virtual double get_initial_eps() override {
        return get_solution_eps();
    }

    virtual double get_initial_eps_planning_time() override;

    virtual double get_final_eps_planning_time() override {
        return get_initial_eps_planning_time;
    }

    virtual int get_n_expands_init_solution() override {
        return get_n_expands();
    }

    virtual double get_final_epsilon() override {
        return get_initial_eps();
    }

    virtual void get_search_stats(std::vector<PlannerStats>* s) override { }
    //}

    int num_heuristics() const { return m_h_count; }

    private:

    using MRMHASearchState_t = MRMHASearchState<N, R>;

    Heuristic* m_h_anchor;
    Heuristic** m_h_inads;
    int m_h_count; // Num of inad heurs + 1

    std::array<int, N> m_rep_ids;
    std::array<std::array<int, N>, N> m_rep_dependency_matrix;

    ReplanParams m_params;
    double m_eps;
    double m_eps_mha;

    MRMHASearchState_t* m_start_state;
    MRMHASearchState_t* m_goal_state;
    std::vector<MRMHASearchState_t> m_search_states;
    CHeap* m_open; ///< sequence of (m_h_count) open lists

    int m_num_expansions;
    double m_time_elapsed;

    MRMHASearchState_t* get_state(int state_id);
    void init_state(MRMHASearchState_t* state, size_t mha_state_idx, int state_id);
    void reinit_state(MRMHASearchState_t* state);

    void reinit_search() { clear_open_lists(); }

    void clear_open_lists();
    void clear();
    int compute_key(MRMHASearchState_t* state, int hidx);

    MRMHASearchState_t* state_from_open_state(MRMHASearchState_t::HeapData* open_state){
        return open_state->me;
    }

    int compute_heuristic(int state_id, int hidx);
    void expand(MRMHASearchState_t* state, int hidx);

    int get_minf(CHeap& pq) const {
        return pq.min()->f;
    }
    void insert_or_update( MRMHASearchState_t* state, int hidx );

    void extract_path(std::vector<int>* solution_path, int* solcost);

    bool closed_in_anc_search(MRMHASearchState_t* state) const {
        return state->closed_in_anc;
    }

    bool closed_in_add_search(MRMHASearchState_t* state, int rep_id) const {
        return state->closed_in_adds[i];
    }

    bool closed_in_any_search(MRMHASearchState_t* state) const {
        return state->closed_in_anc|| std::any(state->closed_in_adds);
    }


};

#include "detail/mrmhaplanner.hpp"

#endif
