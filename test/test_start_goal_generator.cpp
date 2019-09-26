#include "utils/start_goal_generator.h"

int main(){
    Region region;
    std::vector<double> lo1(6, 0), hi1(6, 1);
    std::vector<double> lo2(6, 2), hi2(6, 3);
    region.subregions.push_back(std::make_pair(lo1, hi1));
    region.subregions.push_back(std::make_pair(lo2, hi2));

    std::vector<double> valid_state1(6, 0.5);
    std::vector<double> valid_state2(6, 2.5);
    std::vector<double> invalid_state(6, 4);

    if(region.isValid(valid_state1))
        std::cout<<"Success\n";
    else
        std::cout<<"Failure\n";
    if(region.isValid(valid_state2))
        std::cout<<"Success\n";
    else
        std::cout<<"Failure\n";
    if(!region.isValid(invalid_state))
        std::cout<<"Success\n";
    else
        std::cout<<"Failure\n";
}
