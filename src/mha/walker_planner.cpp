#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <memory>
#include <fstream>
#include <algorithm>

#include <ros/ros.h>
#include <smpl/graph/manip_lattice_action_space.h>
//#include <smpl/graph/manip_lattice_multi_rep.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/heuristic/bfs_2d_heuristic.h>
#include <smpl/heuristic/bfs_3d_heuristic.h>
#include <smpl/heuristic/bfs_3d_base_heuristic.h>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/heuristic/bfs_base_rot_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/euclid_diffdrive_heuristic.h>
#include <smpl/heuristic/arm_retract_heuristic.h>
#include <smpl/heuristic/mother_heuristic.h>
#include <sbpl/planners/mhaplanner.h>

#include "motion_planner.h"
#include "motion_planner_ros.h"
#include "utils/utils.h"

#include "walker_planner/GraspPose.h"
#include "walker_planner/Path1.h"

using Path = std::vector< std::array<double, 3> >;
using Point = std::pair<double, double>;

using namespace std;

struct PlanFeatures {
    std::vector<double> x_rel_door;
    std::vector<double> y_rel_door;
    std::vector<bool> base_path_through_door;
};

bool constructHeuristics(
        std::vector<std::unique_ptr<smpl::RobotHeuristic>>& heurs,
        smpl::ManipLattice* pspace,
        smpl::OccupancyGrid* grid,
        smpl::KDLRobotModel* rm,
        PlannerConfig& params ){

    SMPL_INFO("Initialize Heuristics");
    const int DefaultCostMultiplier = 1000;

    struct AnchorHeuristic : public BfsHeuristic {
        int GetGoalHeuristic(int state_id) override {
            return std::max(bfs_3d_base->GetGoalHeuristic(state_id), bfs_3d->GetGoalHeuristic(state_id));
        }

        double getMetricGoalDistance(double x, double y, double z) override {
            return bfs_3d->getMetricGoalDistance(x, y, z);
        }

        double getMetricStartDistance(double x, double y, double z) override {
            return bfs_3d->getMetricStartDistance(x, y, z);
        }
    };

    /*
    struct EndEffHeuristic : public BfsHeuristic {
        bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
                std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d){
            if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
                return false;
            pose_ext = bfs_3d->planningSpace()->getExtension<smpl::PoseProjectionExtension>();
            return true;
        }

        int GetGoalHeuristic(int state_id){
            if (state_id == bfs_3d->planningSpace()->getGoalStateID()) {
                return 0;
            }
            if(pose_ext == nullptr)
                return 0;
            smpl::Affine3 p;
            if(!pose_ext->projectToPose(state_id, p))
                return 0;

            auto goal_pose = bfs_3d->planningSpace()->goal().pose;

            smpl::Quaternion qa(p.rotation());
            smpl::Quaternion qb(goal_pose.rotation());
            double dot = qa.dot(qb);
            if (dot < 0.0) {
                qb = smpl::Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
                dot = qa.dot(qb);
            }
            int rot_dist = DefaultCostMultiplier*smpl::angles::normalize_angle(2.0 * std::acos(dot));

            int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
            int arm_dist = bfs_3d->GetGoalHeuristic(state_id);

            int heuristic = base_coeff*base_dist + arm_coeff*arm_dist + rot_coeff*rot_dist;
            ROS_ERROR("%d + %d + %d = %d", int(base_coeff*base_dist), int(arm_coeff*arm_dist), int(rot_coeff*rot_dist), heuristic);
            return heuristic;
        }

        double base_coeff=0.02;
        double arm_coeff=1;
        double rot_coeff=1;
        smpl::PoseProjectionExtension* pose_ext = nullptr;
    };
    */

    struct RetractArmHeuristic : public BfsHeuristic {
        bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
                std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d){
            if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
                return false;
            return true;
        }

        int GetGoalHeuristic(int state_id){
            if(state_id == 0)
                return 0;

            smpl::Vector3 p;
            if(!bfs_3d->m_pp->projectToPoint(state_id, p)){
                SMPL_ERROR("RetractArmHeuristic Could not project");
                return 0;
            }
            auto retracted_robot_state = bfs_3d->planningSpace()->getExtension<smpl::ExtractRobotStateExtension>()->extractState(state_id);

            /* End-effector distance
            for(int i=3; i<retracted_robot_state.size(); i++)
                retracted_robot_state[i] = 0;
            smpl::Vector3 retracted_p;
            if(!bfs_3d->m_pp->projectToPoint(retracted_robot_state, retracted_p)){
                SMPL_ERROR("RetractArmHeuristic Could not project");
                return 0;
            }
            int retract_heuristic = euclidDist(p.data(), retracted_p.data(), 3) * DefaultCostMultiplier;
            */

            // Norm of first 5 joints.
            double norm = 0.0;
            for(int i=3; i<8; i++)
                norm += (retracted_robot_state[i]*retracted_robot_state[i]);
            norm = sqrt(norm);
            int retract_heuristic = DefaultCostMultiplier * norm;

            int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);

            int heuristic = base_coeff*base_dist + retract_arm_coeff*retract_heuristic;
            //ROS_ERROR("%d + %d = %d", base_dist, int(retract_arm_coeff*retract_heuristic), heuristic);

            return heuristic;
        }

        double base_coeff = 0.0;
        double retract_arm_coeff = 10.0;

    };

    struct ImprovedEndEffHeuristic : public BfsHeuristic {
        bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
                std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d,
                std::shared_ptr<RetractArmHeuristic> _retract_arm){
            if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
                return false;
            m_retract_arm_heur = _retract_arm;
            pose_ext = bfs_3d->planningSpace()->getExtension<smpl::PoseProjectionExtension>();
            return true;
        }

        int GetGoalHeuristic(int state_id){
            if (state_id == bfs_3d->planningSpace()->getGoalStateID()) {
                return 0;
            }
            if(pose_ext == nullptr)
                return 0;
            smpl::Affine3 p;
            if(!pose_ext->projectToPose(state_id, p))
                return 0;

            auto goal_pose = bfs_3d->planningSpace()->goal().pose;

            smpl::Quaternion qa(p.rotation());
            smpl::Quaternion qb(goal_pose.rotation());
            double dot = qa.dot(qb);
            if (dot < 0.0) {
                qb = smpl::Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
                dot = qa.dot(qb);
            }
            int rot_dist = DefaultCostMultiplier*smpl::angles::normalize_angle(2.0 * std::acos(dot));

            int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
            int arm_dist  = 0;
            //if(arm_dist > 10000){
                //arm_dist = m_retract_arm_heur->GetGoalHeuristic(state_id);
                //rot_dist = 0.0;
            //} else {
                arm_dist = bfs_3d->GetGoalHeuristic(state_id);
            //}

            //int heuristic = base_coeff*base_dist + arm_coeff*arm_dist + rot_coeff*rot_dist;
            int heuristic = arm_coeff*arm_dist + rot_coeff*rot_dist;
            //ROS_ERROR("%d + %d + %d = %d", int(base_coeff*base_dist), int(arm_coeff*arm_dist), int(rot_coeff*rot_dist), heuristic);
            return heuristic;
        }

        double base_coeff=0.05;
        double arm_coeff=1;
        double rot_coeff=2;

        std::shared_ptr<RetractArmHeuristic> m_retract_arm_heur;
        smpl::PoseProjectionExtension* pose_ext = nullptr;
    };

    struct BaseRotHeuristic : public BfsHeuristic {
        bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
                std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d,
                std::shared_ptr<RetractArmHeuristic> _retract_arm,
                double _orientation){
            if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
                return false;
            m_retract_arm_heur = _retract_arm;
            orientation = _orientation;
            return true;
        }

        int GetGoalHeuristic(int state_id){
            if(state_id == 0)
                return 0;
	    ROS_ERROR("Dynamic casting");
            auto robot_state = (dynamic_cast<smpl::ManipLattice*>(
                        bfs_3d->planningSpace()))->extractState(state_id);
            int yaw_dist = DefaultCostMultiplier*smpl::angles::shortest_angle_dist(robot_state[2], orientation);

            int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
            double arm_fold_heur = 0.0;
            if(base_dist > 10000)
                arm_fold_heur = m_retract_arm_heur->GetGoalHeuristic(state_id);

            int heuristic = base_dist + orientation_coeff*yaw_dist + arm_fold_coeff*arm_fold_heur;
            //ROS_ERROR("%d + %d + %d = %d", int(base_dist), int(orientation_coeff*yaw_dist), int(arm_fold_coeff*arm_fold_heur), heuristic);
            return heuristic;
        }

        std::shared_ptr<RetractArmHeuristic> m_retract_arm_heur;
        double orientation = 0.0;
        double orientation_coeff = 5.0;
        double arm_fold_coeff = 1.0;
    };


/*
    auto bfs_2d = std::make_unique<smpl::Bfs2DHeuristic>();
    bfs_2d->setCostPerCell(params.cost_per_cell);
    bfs_2d->setInflationRadius(params.inflation_radius_2d);
    if (!bfs_2d->init(pspace, grid)) {
        ROS_ERROR("Could not initialize Bfs2Dheuristic.");
        return false;
    }
*/

    auto bfs_3d = std::make_shared<smpl::Bfs3DHeuristic>();
    bfs_3d->setCostPerCell(params.cost_per_cell);
    bfs_3d->setInflationRadius(params.inflation_radius_3d);
    if (!bfs_3d->init(pspace, grid)) {
        ROS_ERROR("Could not initialize Bfs3Dheuristic.");
        return false;
    }

    auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
    bfs_3d_base->setCostPerCell(params.cost_per_cell);
    bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
    if (!bfs_3d_base->init(pspace, grid, 4)) {
        ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
        return false;
    }

    auto retract_arm = std::make_shared<RetractArmHeuristic>();
    if(!retract_arm->init(bfs_3d_base, bfs_3d)){
        ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
        return false;
    }


    //Compute a feasible base location.
    std::vector<int> base_x, base_y;
    heurs.clear();

    {
        auto anchor = std::make_unique<AnchorHeuristic>();
        anchor->init( bfs_3d_base, bfs_3d );
        heurs.push_back(std::move(anchor));
    }
    /*
    {
        auto inad = std::make_unique<EndEffHeuristic>();
        inad->init( bfs_3d_base, bfs_3d );
        heurs.push_back(std::move(inad));
    }*/
    {
        auto inad = std::make_unique<ImprovedEndEffHeuristic>();
        if(!inad->init( bfs_3d_base, bfs_3d, retract_arm )){
            ROS_ERROR("Could not initialize ImprovedEndEffHeuristic.");
            return false;
        }
        heurs.push_back(std::move(inad));
    }
    /*
    {
        auto h = std::make_unique<smpl::Bfs3DHeuristic>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace, grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        heurs.push_back(std::move(h));
    }
    */

    int num_rot_heurs = 0;
    for(int i=0; i<num_rot_heurs; i++){
        auto h = std::make_unique<BaseRotHeuristic>();
        if (!h->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
            ROS_ERROR("Could not initialize BaseRotheuristic.");
            return false;
        }
        heurs.push_back(std::move(h));
    }
    {
        auto h = std::make_unique<smpl::EuclidDiffHeuristic>();
        if (!h->init(pspace)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        //heurs.push_back(std::move(h));
    }
    /*
    {
        auto h = std::make_unique<smpl::BfsFullbodyHeuristic>();
        h->setCostPerCell(params.cost_per_cell);
        h->setInflationRadius(params.inflation_radius);
        if (!h->init(pspace, grid)) {
            ROS_ERROR("Could not initialize heuristic.");
            return false;
        }
        //SV_SHOW_INFO(h->get2DMapVisualization());
        //heurs.push_back(std::move(h));
    }
    */

    for (auto& entry : heurs) {
        pspace->insertHeuristic(entry.get());
    }
    return true;
}

class ReadExperimentFromParamServer {
    public:
    ReadExperimentFromParamServer(ros::NodeHandle);
    moveit_msgs::RobotState getStart(PlanningEpisode);
    smpl::GoalConstraint getGoal(PlanningEpisode);
    bool canCallPlanner() const;

    private:
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped start){
        geometry_msgs::Pose start_base = start.pose.pose;
        moveit_msgs::RobotState start_state;
        if (!ReadInitialConfiguration(m_nh, start_state)) {
            ROS_ERROR("Failed to get initial configuration.");
            return;
        }

        start_state.joint_state.position[0] = start_base.position.x;
        start_state.joint_state.position[1] = start_base.position.y;

        {
            double roll, pitch, yaw;
            tf::Quaternion q(
                start_base.orientation.x,
                start_base.orientation.y,
                start_base.orientation.z,
                start_base.orientation.w);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            start_state.joint_state.position[2] = yaw;
        }
        m_start_state = start_state;
        m_start_received = true;
    }

    void goalCallback(const geometry_msgs::PoseStamped _grasp){
        geometry_msgs::Pose grasp = _grasp.pose;

        smpl::Affine3 goal_pose;
        tf::poseMsgToEigen( grasp, goal_pose);

        smpl::GoalConstraint goal;
        goal.type = smpl::GoalType::XYZ_RPY_GOAL;
        goal.pose = goal_pose;
        goal.xyz_tolerance[0] = 0.03;
        goal.xyz_tolerance[1] = 0.03;
        goal.xyz_tolerance[2] = 0.03;
        goal.rpy_tolerance[0] = 1.70;
        goal.rpy_tolerance[1] = 1.70;
        goal.rpy_tolerance[2] = 1.70;

        m_goal_constraint = goal;
        m_goal_received = true;
    }

    ros::NodeHandle m_nh;
    ros::Subscriber m_sub_goal;
    ros::Subscriber m_sub_start;
    moveit_msgs::RobotState m_start_state;
    smpl::GoalConstraint m_goal_constraint;
    bool m_start_received = false;
    bool m_goal_received = false;
    std::vector<bool*> m_status_variables;
};

ReadExperimentFromParamServer::ReadExperimentFromParamServer(ros::NodeHandle nh){
    m_nh = nh;
    m_sub_start = m_nh.subscribe("/poseupdate", 1000, &ReadExperimentFromParamServer::startCallback, this);
    m_sub_goal = m_nh.subscribe("/Grasps", 1000, &ReadExperimentFromParamServer::goalCallback, this);
    m_status_variables = {
        &m_start_received,
        &m_goal_received
    };
}

moveit_msgs::RobotState ReadExperimentFromParamServer::getStart(PlanningEpisode _ep){
    return m_start_state;
}

smpl::GoalConstraint ReadExperimentFromParamServer::getGoal(PlanningEpisode _ep){
    return m_goal_constraint;
}

bool ReadExperimentFromParamServer::canCallPlanner() const {
    return std::all_of(
            m_status_variables.begin(), m_status_variables.end(),
            [](bool* x){
                return *x;
                }
            );
}

bool writePath(std::string _file_name, std::string _header, std::vector<smpl::RobotState> _path){
    std::ofstream file;
    file.open(_file_name, std::ios::out);
    if(!file.is_open()){
        SMPL_ERROR("Could not open file.");
        return false;
    }

    file<<_header<<"\n";

    for(const auto& state : _path){
        for(const auto& val : state){
            file<<val<<" ";
        }
        file<<"\n";
    }

    file.close();
    return true;
}

void publish_path(std::vector<smpl::RobotState> path, ros::Publisher path_pub) {
  ROS_ERROR("publishing path");
  int number_of_joints = 16;
  ros::Rate r(10);

  for(int k=0; k<10000;k++){
  walker_planner::Path1 Final_path;
  for(int i=0;i<path.size();i++){
      walker_planner::GraspPose State;
      for(int j=0;j<path[i].size();j++){
          State.a.push_back(path[i][j]);
      }
      Final_path.path.push_back(State);
    }
    path_pub.publish(Final_path);
  }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "walker_planner");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Rate loop_rate(10);
    smpl::VisualizerROS visualizer(nh, 100);
    smpl::viz::set_visualizer(&visualizer);

    /////////////////
    // Robot Model //
    /////////////////

    ROS_INFO("Load common parameters");
    auto robot_description_key = "robot_description";
    std::string robot_description_param;
    if (!nh.searchParam(robot_description_key, robot_description_param)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return 1;
    }

    std::string robot_description;
    if (!nh.getParam(robot_description_param, robot_description)) {
        ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
        return 1;
    }

    // Reads planning_joints, frames.
    RobotModelConfig robot_config;
    if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model"), robot_config)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return 1;
    }

    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    ROS_INFO("Initialize Occupancy Grid");

    auto df_size_x = 20.0;
    auto df_size_y = 15.0;
    auto df_size_z = 1.5;
    auto df_res = 0.05;
    auto df_origin_x = -10;//0;
    auto df_origin_y = -10;//0;
    auto df_origin_z = 0.0;
    auto max_distance = 1.8;

    using DistanceMapType = smpl::EuclidDistanceMap;

    auto df = std::make_shared<DistanceMapType>(
            df_origin_x, df_origin_y, df_origin_z,
            df_size_x, df_size_y, df_size_z,
            df_res,
            max_distance);

    auto ref_counted = false;
    auto grid_ptr = std::make_unique<smpl::OccupancyGrid>(df, ref_counted);

    grid_ptr->setReferenceFrame(planning_frame);
    SV_SHOW_INFO(grid_ptr->getBoundingBoxVisualization());

    //////////////////////////////////
    // Initialize Collision Checker //
    //////////////////////////////////

    ROS_INFO("Initialize collision checker");

    // This whole manage storage for all the scene objects and must outlive
    // its associated CollisionSpace instance.
    //auto scene_ptr = std::make_unique<CollisionSpaceScene>();
    auto scene_ptr = std::make_unique<CollisionSpaceScene>();

    smpl::collision::CollisionModelConfig cc_conf;
    if (!smpl::collision::CollisionModelConfig::Load(ph, cc_conf)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return 1;
    }

    ROS_INFO("collision model loaded");
    smpl::collision::CollisionSpace cc;
    if (!cc.init(
            grid_ptr.get(),
            robot_description,
            cc_conf,
            robot_config.group_name,
            robot_config.planning_joints))
    {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }
    cc.setPadding(0.02);
    cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

    //auto marker = cc.getCollisionRobotVisualization(start_state.joint_state.position);
    //marker[0].ns = "Start state";
    //SV_SHOW_INFO(marker);

    /////////////////
    // Setup Scene //
    /////////////////

    PlannerConfig planning_config;
    if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)){
        ROS_ERROR("Failed to read planner config");
        return 1;
    }
    ROS_INFO("Initialize scene");

    scene_ptr->SetCollisionSpace(&cc);

    ROS_INFO("Setting up robot model");
    auto rm = SetupRobotModel(robot_description, robot_config);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    rm->printRobotModelInformation();

    SV_SHOW_INFO(grid_ptr->getDistanceFieldVisualization(0.2));

    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());

    ros::Duration(1.0).sleep();

    auto resolutions = getResolutions( rm.get(), planning_config );
    auto action_space = std::make_unique<smpl::ManipLatticeActionSpace>();
    auto space = std::make_unique<smpl::ManipLattice>();

    if (!space->init( rm.get(), &cc, resolutions, action_space.get() )) {
        SMPL_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }

    if (!action_space->init(space.get())) {
        SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
        return 1;
    }

    if(!action_space->load(planning_config.mprim_filename))
        return 1;

    space->setVisualizationFrameId(grid_ptr->getReferenceFrame());

    using MotionPrimitive = smpl::MotionPrimitive;
    action_space->useMultipleIkSolutions(planning_config.use_multiple_ik_solutions);
    action_space->useAmp(MotionPrimitive::SNAP_TO_XYZ, planning_config.use_xyz_snap_mprim);
    action_space->useAmp(MotionPrimitive::SNAP_TO_RPY, planning_config.use_rpy_snap_mprim);
    action_space->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.use_xyzrpy_snap_mprim);
    action_space->useAmp(MotionPrimitive::SHORT_DISTANCE, planning_config.use_short_dist_mprims);
    action_space->ampThresh(MotionPrimitive::SNAP_TO_XYZ, planning_config.xyz_snap_dist_thresh);
    action_space->ampThresh(MotionPrimitive::SNAP_TO_RPY, planning_config.rpy_snap_dist_thresh);
    action_space->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.xyzrpy_snap_dist_thresh);
    action_space->ampThresh(MotionPrimitive::SHORT_DISTANCE, planning_config.short_dist_mprims_thresh);

    ///////////////
    //Planning////
    /////////////


    std::vector<std::unique_ptr<smpl::RobotHeuristic>> robot_heurs;

    if(!constructHeuristics( robot_heurs, space.get(), grid_ptr.get(), rm.get(), planning_config )){
        ROS_ERROR("Could not construct heuristics.");
        return 0;
    }

    ROS_ERROR("Num heurs: %d", robot_heurs.size());
    assert(robot_heurs[0] != nullptr);

    std::vector<Heuristic*> heurs;

    for(int i=0; i < robot_heurs.size(); i++)
        heurs.push_back(robot_heurs[i].get());

    Heuristic* anchor_heur = heurs[0];
    std::vector<Heuristic*> inad_heurs( heurs.begin() + 1, heurs.end() );

    using MotionPlanner = MPlanner::MotionPlanner<MHAPlanner, smpl::ManipLattice>;
    auto search_ptr = std::make_unique<MHAPlanner>(
            space.get(), anchor_heur, inad_heurs.data(), inad_heurs.size());

    ROS_ERROR("%f", planning_config.cost_per_cell);
    ROS_ERROR("%f", planning_config.inflation_radius_2d);
    ROS_ERROR("%f", planning_config.inflation_radius_3d);
    ROS_ERROR("%f", planning_config.eps);
    ROS_ERROR("%f", planning_config.eps_mha);

    const int max_planning_time = planning_config.planning_time;
    const double eps = planning_config.eps;
    const double eps_mha = planning_config.eps_mha;
    MPlanner::PlannerParams planner_params = { max_planning_time, eps, eps_mha, false };

    auto mplanner = std::make_unique<MotionPlanner>();
    mplanner->init(search_ptr.get(), space.get(), heurs, planner_params);

    MotionPlannerROS< Callbacks<>, ReadExperimentFromParamServer, MotionPlanner >
            mplanner_ros(ph, rm.get(), scene_ptr.get(), mplanner.get(), grid_ptr.get());

    ExecutionStatus status = ExecutionStatus::WAITING;
    //while(status == ExecutionStatus::WAITING) {
    std::string file_prefix = "paths/solution_path";
    std::ofstream stats_file;
    PlanningEpisode ep = planning_config.start_planning_episode;
    auto path_pub = nh.advertise<walker_planner::Path1>("Robot_path", 1000);

    std::vector<smpl::RobotState> plan;
    while(ep <= planning_config.end_planning_episode ){
        loop_rate.sleep();
        std::string file_suffix = std::to_string(ep) + ".txt";
        space->clearStats();

	int done;
        ros::param::get("/walker_planner_done", done);
        if(done){
            publish_path(plan, path_pub);
	    continue;
	}
        status = mplanner_ros.execute(ep);
    //}
        if(status == ExecutionStatus::SUCCESS){
            ROS_INFO("----------------");
            ROS_INFO("Planning Time: %f", mplanner_ros.getPlan(ep).planning_time);
            ROS_INFO("----------------");
            plan = mplanner_ros.getPlan(ep).robot_states;

            // Write to file.
            stats_file.open("planning_stats.txt", std::ios::app);
            auto plan_stats = mplanner_ros.getPlan(ep);
            stats_file<<std::to_string(ep)<<" "<<max_planning_time<<" ";
            stats_file<<plan_stats.planning_time << " " << plan_stats.num_expansions << " " << plan_stats.cost<<" ";
            stats_file<<plan_stats.ik_computations<<" "<<plan_stats.ik_evaluations<<" "<<plan_stats.ik_valid<<" ";
            stats_file<<std::to_string(eps)<<" "<<std::to_string(eps_mha)<<"\n";
            stats_file.close();

            std::string file_name = file_prefix + file_suffix;
            std::string header = "Solution Path";
            SMPL_ERROR("%s", file_name.c_str());
            writePath(file_name, header , plan);
            ///////////////////
            //Compute Features
            //////////////////
            /*
            std::vector<moveit_msgs::CollisionObject> landmarks;
            auto map_config = getMultiRoomMapConfig(ph);
            auto objects = GetMultiRoomMapCollisionCubes(grid_ptr->getReferenceFrame(), map_config, landmarks );
            ROS_ERROR("Landmarks: %d", landmarks.size());
            auto features = computePlanFeatures(plan_stats,
                    dynamic_cast<BfsHeuristic*>(anchor_heur)->bfs_3d_base.get(),
                    landmarks[0]);
            ROS_ERROR("Features size: %d", features.x_rel_door.size());
            for(int i=0; i<features.x_rel_door.size(); i++){
                ROS_ERROR("%f", features.x_rel_door[i]);
                ROS_ERROR("%f", features.y_rel_door[i]);
                ROS_ERROR("%d", features.base_path_through_door[i]);
            }
            */


            visualization_msgs::MarkerArray whole_path;
            std::vector<visualization_msgs::Marker> m_all;

            ///*
            int idx = 0;
            for( int pidx=0; pidx<plan.size(); pidx++ ){
                auto& state = plan[pidx];
                auto markers = cc.getCollisionRobotVisualization(state);
                for (auto& m : markers.markers) {
                    m.ns = "path_animation";
                    m.id = idx;
                    idx++;
                    whole_path.markers.push_back(m);
                }
                visualizer.visualize(smpl::visual::Level::Info, markers);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            //*/
	    publish_path(plan, path_pub);
            ros::param::set("/walker_planner_done", 1);
        }

        if(status != ExecutionStatus::WAITING){
            status = ExecutionStatus::WAITING;
            ep++;
        }
        ros::spinOnce();
    }

}
