#ifndef WALKER_MHAPLANNER_SINGLE_PARAM_H
#define WALKER_MHAPLANNER_SINGLE_PARAM_H

#include "mrmhaplanner.h"

template <int N, typename SP>
class MHAPlannerSingleParam : public MRMHAPlanner<N, 1, SP>
{
    public:
    MHAPlannerSingleParam( DiscreteSpaceInformation* env,
        std::array<Heuristic*, N>& heurs,
        std::array<int, N>& rep_ids,
        std::array<std::array<int, 1>,1>& rep_dependency_matrix,
        SP* scheduling_policy );

    protected:
    using typename MRMHAPlanner<N, 1, SP>::MRMHASearchState;
    int compute_key(MRMHASearchState* state, int hidx) override;
};

template <int N, typename SP>
MHAPlannerSingleParam<N, SP>::MHAPlannerSingleParam( DiscreteSpaceInformation* _env,
        std::array<Heuristic*, N>& _heurs,
        std::array<int, N>& _rep_ids,
        std::array<std::array<int, 1>,1>& _rep_dependency_matrix,
        SP* _scheduling_policy ) :
    MRMHAPlanner<N, 1, SP>( _env, _heurs, _rep_ids,
            _rep_dependency_matrix, _scheduling_policy ) {}

template <int N, typename SP>
int MHAPlannerSingleParam<N, SP>::compute_key( MRMHASearchState* _state,
        int _hidx ){
    if( _hidx == 0 )
        return _state->g + (int)_state->od[_hidx].h;
    else
        return _state->g + (int)(this->m_eps * (double)_state->od[_hidx].h);
}

#endif
