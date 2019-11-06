#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen_conversions/eigen_msg.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>

#include "utils/start_goal_generator.h"
#include "utils/utils.h"

#define PI 3.14

bool Region::isValid(std::vector<double> _state){
    assert(subregions.size());
    assert(_state.size() == subregions[0].first.size());

    bool validity = false;
    for(auto& subregion : subregions){
        bool inBound = true;
        auto& lo = subregion.first;
        auto& hi = subregion.second;
        for(int i=0; i<_state.size(); i++){
            if(_state[i] < lo[i] || _state[i] > hi[i]){
                inBound = false;
                break;
            }
        }
        if(inBound){
            validity = true;
            break;
        }
    }

    return validity;
}

std::vector<double> Region::getRandState(){
    const int N = subregions[0].first.size();
    std::vector<double> state(N, 0);
    int subregionIx = rand() % subregions.size();
    return getRandStateInSubregion(subregionIx);
}

std::vector<double> Region::getRandStateInSubregion(int _subregionIx){
    const int N = subregions[0].first.size();
    std::vector<double> state(N, 0);
    auto& lo = subregions[_subregionIx].first;
    auto& hi = subregions[_subregionIx].second;
    for(int i=0; i<state.size(); i++){
        if(fabs(hi[i] - lo[i]) < 0.001)
            state[i] = lo[i];
        else
            state[i] = getRandNum(lo[i], hi[i]);
    }
    return state;
}

