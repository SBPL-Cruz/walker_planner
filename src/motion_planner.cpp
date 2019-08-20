#include <tf2/LinearMath/Quaternion.h>
#include <sstream>

#include <smpl/search/arastar.h>
#include <sbpl/planners/mrmhaplanner.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/bfs_heuristic_rot.h>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/euclid_diffdrive_heuristic.h>
#include <smpl/heuristic/arm_retract_heuristic.h>
#include <smpl/heuristic/base_rot_euclidean_heuristic.h>
#include <smpl/heuristic/base_rot_bfs_heuristic.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/manip_lattice_multi_rep.h>
#include <smpl/utils.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>

#include "motion_planner.h"

using namespace smpl;

bool constructHeuristics(
        std::vector<std::unique_ptr<RobotHeuristic>>& heurs,
        std::unique_ptr<ManipLatticeMultiRep>& pspace,
        smpl::OccupancyGrid& grid,
        std::unique_ptr<smpl::KDLRobotModel>& rm,
        const Eigen::Affine3d& goal,
        PlannerConfig& params ){

    SMPL_INFO("Initialize Heuristics");

    //Compute a feasible base location.
    std::vector<int> base_x, base_y;

    heurs.clear();
    {
        auto h = make_unique<BfsHeuristic>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        heurs.push_back(std::move(h));
    }
    //{
    //    auto h = make_unique<BfsFullbodyHeuristic>();
    //    h->setCostPerCell(params.cost_per_cell);
    //    h->setInflationRadius(params.inflation_radius);
    //    if (!h->init(pspace.get(), &grid)) {
    //        ROS_ERROR("Could not initialize heuristic.");
    //        return false;
    //    }
    //    SV_SHOW_INFO(h->get2DMapVisualization());
    //    heurs.push_back(std::move(h));
    //}

    {
        auto h = make_unique<BfsHeuristicRot>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        heurs.push_back(std::move(h));
    }
    {
        auto h = make_unique<BfsFullbodyHeuristic>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace.get(), &grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        SV_SHOW_INFO(h->get2DMapVisualization());
        heurs.push_back(std::move(h));
    }
    {
        auto h = make_unique<EuclidDiffHeuristic>();
        if (!h->init(pspace.get())) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        h->setWeightRot(0.01);
        //heurs.push_back(std::move(h));
    }


    {
        auto h = make_unique<EuclidDistHeuristic>();
        if (!h->init(pspace.get())) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        h->setWeightRot(0.01);
        //heurs.push_back(std::move(h));
    }
    /*
    {
        auto h = make_unique<ArmRetractHeuristic>();
        if (!h->init(pspace.get())) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        heurs.push_back(std::move(h));
    }
    */

    {
        auto h = make_unique<BaseRotEuclideanHeuristic>();
        if (!h->init(pspace.get(), 1.57)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        h->setWeightRot(0.01);
        //heurs.push_back(std::move(h));
    }

    {
        auto h = make_unique<BaseRotBfsHeuristic>();
        if (!h->init(pspace.get(), &grid, 1.57)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        //heurs.push_back(std::move(h));
    }
    for (auto& entry : heurs) {
        pspace->insertHeuristic(entry.get());
    }
    return true;
}
/*
void visualizeRadiusAroundGoal(int x0, int y0, int radius) {
    std::vector<int> circle_x;
    std::vector<int> circle_y;
    double res = m_occupancy_grid->getResolution();
    int discrete_radius = m_radius/res;
    getBresenhamCirclePoints(x0, y0, discrete_radius, circle_x, circle_y);

    // geometry_msgs::PolygonStamped circle;

    // circle.header.frame_id = "/map";
    // circle.header.stamp = ros::Time::now();
    std::vector<geometry_msgs::Point> circle_points;

    for (size_t i = 0; i < circle_x.size(); ++i) {
        // Prune the points to display only the ones that are within the
        // threshold
        if (m_grid[circle_x[i]][circle_y[i]] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            geometry_msgs::Point out_pt;
            out_pt.x = circle_x[i]*res;
            out_pt.y = circle_y[i]*res;
            out_pt.z = 0.0;
            circle_points.push_back(out_pt);
        }
    }
    std::stringstream ss;
    ss<< "radius_around_goal";
    Visualizer::pviz->visualizeLine(
        circle_points, ss.str(), x0 + y0, 114, 0.01);
}
*/

void getBresenhamCirclePoints( int x0,
        int y0,
        int radius,
        std::vector<int>& ret_x,
        std::vector<int>& ret_y ){
    int x = 0;
    int y = radius;
    int delta = 2 - 2 * radius;
    int err = 0;
    ret_x.clear();
    ret_y.clear();
    while(y >= 0){
        ret_x.push_back(x0 + x);
        ret_x.push_back(x0 - x);
        ret_x.push_back(x0 + x);
        ret_x.push_back(x0 - x);
        ret_y.push_back(y0 - y);
        ret_y.push_back(y0 - y);
        ret_y.push_back(y0 + y);
        ret_y.push_back(y0 + y);
        err = 2 * (delta + y) - 1;
        if(delta < 0 && err <= 0){
                x = x + 1;
                delta = delta + 2 * x + 1;
                continue;
        }
        err = 2 * (delta - x) - 1;
        if(delta > 0 && err > 0){
                y = y - 1;
                delta = delta + 1 - 2 * y;
                continue;
        }
        x = x + 1;
        delta = delta + 2 * (x - y);
        y = y - 1;
    }
}
