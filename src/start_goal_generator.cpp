#include <ros/ros.h>
#include <start_goal_generator.h>

double getRandNum(const double _lo, const double _hi ){
    return _lo + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(_hi-_lo)));
}

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
    auto& lo = subregions[subregionIx].first;
    auto& hi = subregions[subregionIx].second;
    for(int i=0; i<state.size(); i++){
        if(fabs(hi[i] - lo[i]) < 0.001)
            state[i] = lo[i];
        else
            state[i] = getRandNum(lo[i], hi[i]);
    }

    return state;
}

bool StartGoalGenerator::init( smpl::collision::CollisionSpace* _cc,
        smpl::urdf::URDFRobotModel* _rm,
        unsigned int _seed ){
    m_cc = _cc;
    m_rm = _rm;
    srand(_seed);
    return true;
}

bool StartGoalGenerator::addStartRegion(BoundedRegion& _start){
    m_start_region.subregions.push_back(std::make_pair(_start.lo, _start.hi));
    return true;
}

bool StartGoalGenerator::addGoalRegion(BoundedRegion& _goal){
    m_goal_region.subregions.push_back(std::make_pair(_goal.lo, _goal.hi));
    return true;
}

bool StartGoalGenerator::generate(int _n){
    assert(m_start_region.subregions.size());
    assert(m_goal_region.subregions.size());
    int num_generated = 0;
    int iters = 0;
    const int max_iters = 1000;
    while(num_generated < _n && iters < max_iters){
        iters++;
        //Start
        auto rand_start = m_start_region.getRandState();
        //if(iters % 1 == 0){
        //    ROS_INFO("Iter: %d", iters);
        //    for(auto& val : rand_start)
        //        ROS_INFO("%f ", val);
        //}

        if(!m_cc->isStateValid(rand_start))
            continue;

        //Goal
        //auto rand_goal = m_goal_region.getRandState();
        //int gx, gy, gz;
        //std::cout<<rand_goal[0]<<" "<<rand_goal[1]<<" "<<rand_goal[2]<<"\n";
        //ROS_ERROR("%f, %f, %f", rand_goal[0], rand_goal[1], rand_goal[2]);
        //m_cc->grid()->worldToGrid(rand_goal[0], rand_goal[1], rand_goal[2],
        //        gx, gy, gz);
        //ROS_ERROR("%d, %d, %d", gx, gy, gz);
        //if(m_cc->grid()->getDistance(gx, gy, gz) <= 0)
        //    continue;

        m_start_states.push_back(rand_start);
        //m_goal_poses.push_back(rand_goal);
        num_generated++;
    }
    ROS_WARN("Num generated: %d", num_generated);
    if(num_generated != _n)
        return false;
    else
        return true;
}

bool StartGoalGenerator::writeToFile(std::string _start_file_name, std::string _goal_file_name){
    {
        std::ofstream start_stream;
        start_stream.open(_start_file_name);
        if(!start_stream.is_open()){
            ROS_ERROR("Could not open start file.");
            return false;
        }
        char header[] = "Start states\n";
        start_stream << header;
        for(auto& state : m_start_states){
            for(auto& val : state){
                start_stream << val <<" ";
            }
            start_stream <<"\n";
        }
        start_stream.close();
    }
    /*
    {
        std::ofstream goal_stream;
        goal_stream.is_open(_goal_file_name);
        (!goal_stream.open())
            throw "Could not open goal file.";
        char header[100] = "Goal Poses\n";
        goal_stream << header;
    }
    */

    return true;
}

void StartGoalGenerator::clear(){
    m_start_region.subregions.clear();
    m_goal_region.subregions.clear();
    m_start_states.clear();
    m_goal_poses.clear();
}
