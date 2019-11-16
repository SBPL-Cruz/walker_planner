#ifndef WALKER_MRMHAPLANNER_H
#define WALKER_MRMHAPLANNER_H

#include <algorithm>
#include <array>

#include <sbpl/planners/planner.h>
#include <sbpl/heuristics/heuristic.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/console/console.h>

#define INFINITECOST std::numeric_limits<int>::max()

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
        std::array<Heuristic*, N>& heurs,
        std::array<int, N>& rep_ids,
        std::array<std::array<int, R>, R>& rep_dependency_matrix,
        SP* scheduling_policy );

    virtual ~MRMHAPlanner(){
        clear();
        delete[] m_open;
    }

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

    virtual void set_initialsolution_eps(double eps) override {
        m_params.initial_eps = eps;
    }

    int set_initial_eps(double eps) {
        set_initialsolution_eps(eps);
        return true;
    }

    int set_initial_mha_eps(double eps_mha){
        m_eps_mha = eps_mha;
        return true;
    }

    bool setSeed(int seed) {
        m_generator.seed(seed);
    }

    virtual int force_planning_from_scratch() override {
        clear();
    }

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

    int get_num_expansions() const {
        return m_num_expansions;
    }

    virtual double get_initial_eps() override {
        return get_solution_eps();
    }

    virtual double get_initial_eps_planning_time() override {
        return m_time_elapsed;
    }

    virtual double get_final_eps_planning_time() override {
        return get_initial_eps_planning_time();
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

    protected:

    struct MRMHASearchState {
        int call_number;
        int state_id;
        int g;
        MRMHASearchState* bp;
        bool closed_in_anc;
        std::array<int, R> closed_in_adds;

        struct HeapData : public smpl::heap_element {
            MRMHASearchState* me;
            int h;
            int f;
        };

        std::array<HeapData, N> od;
    };

    struct HeapCompare
    {
        bool operator()(
            const typename MRMHASearchState::HeapData& s,
            const typename MRMHASearchState::HeapData& t) const
        {
            return s.f < t.f;
        }
    };

    Heuristic* m_h_anchor;
    Heuristic** m_h_inads;
    int m_h_count; // Num of inad heurs + 1
    SP* m_scheduling_policy;
    std::default_random_engine m_generator;

    std::array<int, N> m_rep_ids;
    std::array<std::array<int, R>, R> m_rep_dependency_matrix;

    ReplanParams m_params;
    int m_call_number;
    double m_eps;
    double m_eps_mha;
    double m_eps_satisfied;

    MRMHASearchState* m_start_state;
    MRMHASearchState* m_goal_state;
    std::vector<MRMHASearchState*> m_search_states;

    typedef smpl::intrusive_heap<typename MRMHASearchState::HeapData, HeapCompare> CHeap;
    CHeap* m_open; ///< sequence of (m_h_count) open lists

    int m_num_expansions;
    int m_max_expansions;
    double m_time_elapsed;

    bool check_params(const ReplanParams& params);
    bool time_limit_reached();

    MRMHASearchState* get_state(int state_id){
        assert(state_id >= 0 && state_id < environment_->StateID2IndexMapping.size());
        int* idxs = environment_->StateID2IndexMapping[state_id];
        if (idxs[MHAMDP_STATEID2IND] == -1) {
            auto s = new MRMHASearchState;
            const size_t mha_state_idx = m_search_states.size();
            init_state(s, mha_state_idx, state_id);

            // map graph state to search state
            idxs[MHAMDP_STATEID2IND] = (int)mha_state_idx;

            m_search_states.push_back(s);

            return s;
            //return &(*m_search_states.rbegin());
        }
        else {
            int ssidx = idxs[MHAMDP_STATEID2IND];
            return m_search_states[ssidx];
        }
    }

    void init_state(MRMHASearchState* state, size_t mha_state_idx, int state_id);
    void reinit_state(MRMHASearchState* state);

    void reinit_search() { clear_open_lists(); }

    void clear_open_lists();
    void clear();
    int compute_key(MRMHASearchState* state, int hidx);

    MRMHASearchState* state_from_open_state(typename MRMHASearchState::HeapData* open_state){
        return open_state->me;
    }

    int sampleIndex(const std::array<double, N>& likelihoods);
    int chooseQueue();
    int compute_heuristic(int state_id, int hidx);
    virtual void expand(MRMHASearchState* state, int hidx);

    int get_minf(CHeap& pq) const {
        return pq.min()->f;
    }
    void insert_or_update( MRMHASearchState* state, int hidx );

    void extract_path(std::vector<int>* solution_path, int* solcost);

    bool closed_in_anc_search(MRMHASearchState* state) const {
        return state->closed_in_anc;
    }

    bool closed_in_add_search(MRMHASearchState* state, int rep_id) const {
        return state->closed_in_adds[rep_id];
    }

    bool closed_in_any_search(MRMHASearchState* state) const {
        return state->closed_in_anc|| std::any_of(state->closed_in_adds);
    }

};

#include "detail/mrmhaplanner.hpp"

#endif
