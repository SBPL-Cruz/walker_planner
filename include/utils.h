#ifndef WALKER_PLANNER_UTILS_H
#define WALKER_PLANNER_UTILS_H

#include <cstdlib>
#include <cmath>
#include <limits>

template <typename T>
inline T getRandNum(const T _lo, const T _hi ){
    return _lo + static_cast <T> (rand()) /( static_cast <T> (RAND_MAX/(_hi -_lo)));
}

template <typename T>
inline double euclidDist( const std::pair<T, T>& a, const std::pair<T, T>& b ){
    return sqrt((a.first - b.first)*(a.first - b.first) + (a.second - b.second)*(a.second - b.second));
}

inline double minPairwiseDist(const std::vector<std::pair<double, double>>& vec){
    double min_dist = std::numeric_limits<double>::max();
    for(int i=0; i<vec.size(); i++){
        for(int j=0; j<vec.size(); j++){
            if(i == j)
                continue;
            double dist = euclidDist(vec[i], vec[j]);
            if( dist < min_dist )
                min_dist = dist;
        }
    }
    return min_dist;
}

inline double minDistFromEach(
        const std::pair<double, double> new_point,
        const std::vector<std::pair<double, double>>& vec ){
    double min_dist = std::numeric_limits<double>::max();
    for(int i=0; i<vec.size(); i++){
        double dist = euclidDist(vec[i], new_point);
        if( dist < min_dist )
            min_dist = dist;
    }
    return min_dist;
}

#endif
