#ifndef WALKER_PLANNER_UTILS_H
#define WALKER_PLANNER_UTILS_H

#include <cstdlib>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <smpl/spatial.h>
#include <smpl/angles.h>

template <typename T>
inline T getRandNum(const T _lo, const T _hi ){
    return _lo + static_cast <T> (rand()) /( static_cast <T> (RAND_MAX/(_hi -_lo)));
}

template <typename T>
inline double euclidDist( const std::pair<T, T>& a, const std::pair<T, T>& b ){
    return sqrt((a.first - b.first)*(a.first - b.first) + (a.second - b.second)*(a.second - b.second));
}

template <typename T>
inline double euclidDist( const T* a, const T* b, unsigned int n ){
    double dist_sqrd = 0.0;
    for(int i=0; i<n; i++)
        dist_sqrd += (a[i] - b[i])*(a[i] - b[i]);
    return sqrt(dist_sqrd);
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

inline bool areClose( const smpl::Affine3& a, const smpl::Affine3& b ){
    auto trans_a = a.translation(), trans_b = b.translation();
    auto rot_a = a.rotation(), rot_b = b.rotation();
    if(euclidDist(trans_a.data(), trans_b.data(), 3) > 0.01)
        return false;
    if(euclidDist(rot_a.data(), rot_b.data(), 4) > 0.01)
        return false;
    return true;
}

inline bool areClose(const double a, const double b){
    return fabs(a - b) < 0.001;
}

std::vector<double> poseToXYZRPY(Eigen::Affine3d& _pose);

#endif
