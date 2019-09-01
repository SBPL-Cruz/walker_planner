#include <start_goal_generator.h>

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

bool StartGoalGenerator::init( smpl::collision::CollisionSpace* _cc,
        smpl::urdf::URDFRobotModel* _rm,
        unsigned int _seed ){
    m_cc = _cc;
    m_rm = _rm;
    srand(_seed);
    return true;
}

bool StartGoalGenerator::addStartRegion(StartConstraint& _start){
    std::vector<double> lo(_start.reference.size(), 0); 
    std::vector<double> hi(_start.reference.size(), 0);
    for(int i=0; i<_start.reference.size(); i++){
        lo[i] = _start.reference[i] - _start.tol[i];
        hi[i] = _start.reference[i] + _start.tol[i];
    }
    m_start_region.subregions.push_back(std::make_pair(lo, hi));
    return true;
}

bool StartGoalGenerator::addGoalRegion(smpl::GoalConstraint& _goal){
    std::vector<double> lo(6, 0), hi(6, 0);
    for(int i=0; i<3; i++){
        lo[i] = _goal.pose.translation()[i] - _goal.xyz_tolerance[i];
        hi[i] = _goal.pose.translation()[i] + _goal.xyz_tolerance[i];
    }

    double rpy[3];
    smpl::get_euler_zyx(_goal.pose.rotation(), rpy[2], rpy[1], rpy[0]);
    for(int i=0; i<3; i++){
        lo[i+3] = rpy[i] - _goal.rpy_tolerance[i];
        hi[i+3] = rpy[i] + _goal.rpy_tolerance[i];
    }
    m_goal_region.subregions.push_back(std::make_pair(lo, hi));
    return true;
}

double StartGoalGenerator::getRandNum(double _lo, double _hi ){
    return _lo + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(_hi-_lo)));
}

void StartGoalGenerator::clear(){
    m_start_region.subregions.clear();
    m_goal_region.subregions.clear();
}
