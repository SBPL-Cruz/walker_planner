#ifndef WALKER_MRMHAPLANNER_CONTEXT_H
#define WALKER_MRMHAPLANNER_CONTEXT_H

#include <array>
#include <vector>

template <int N, typename T>
class AbstractContext {
    public:
    virtual std::array<T, N> getContext(const std::vector<double>& robot_state) = 0;
    virtual std::array<T, N> getContext(int state_id) = 0;
};

#endif
