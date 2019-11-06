#include "utils/utils.h"

std::vector<double> poseToXYZRPY(Eigen::Affine3d& _pose){
    auto trans = _pose.translation();
    auto rot = _pose.rotation();
    double r, p, y;
    smpl::angles::get_euler_zyx(rot, y, p, r);
    std::vector<double> pose_vector;
    for(int i = 0; i < 3; i++)
        pose_vector.push_back(trans[i]);
    pose_vector.push_back(r);
    pose_vector.push_back(p);
    pose_vector.push_back(y);

    return pose_vector;
}
