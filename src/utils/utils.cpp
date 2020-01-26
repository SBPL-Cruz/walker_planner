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

Eigen::Affine3d XYZRPYToPose(std::vector<double> _xyzrpy)
{
    assert(_xyzrpy.size() == 6);
    return Eigen::Affine3d(
            Eigen::Translation3d(_xyzrpy[0], _xyzrpy[1], _xyzrpy[2]) *
            Eigen::AngleAxisd(_xyzrpy[3], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(_xyzrpy[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(_xyzrpy[5], Eigen::Vector3d::UnitX()));
}
