#include "heuristics/walker_heuristics.h"
#include <smpl/spatial.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/visualize.h>

bool constructHeuristics(
        std::array< std::shared_ptr<smpl::RobotHeuristic>, NUM_QUEUES >& heurs,
        std::array<int, NUM_QUEUES>& rep_ids,
        std::vector< std::shared_ptr<smpl::RobotHeuristic> >& bfs_heurs,
        smpl::ManipLattice* pspace,
        smpl::OccupancyGrid* grid,
        smpl::KDLRobotModel* rm,
        PlannerConfig& params ){

    SMPL_INFO("Initialize Heuristics");
    const int DefaultCostMultiplier = 1000;


    auto bfs_3d = std::make_shared<smpl::Bfs3DHeuristic>();
    bfs_3d->setCostPerCell(params.cost_per_cell);
    bfs_3d->setInflationRadius(params.inflation_radius_3d);
    if (!bfs_3d->init(pspace, grid)) {
        ROS_ERROR("Could not initialize Bfs3Dheuristic.");
        return false;
    }

    int hidx = 0;
    ROS_INFO("Creating Fullbody Anchor Heuristic");
    {
        // This won't be admissible
        //auto anchor = std::make_unique<AnchorHeuristic>();
        //anchor->init( bfs_3d_base, bfs_3d );
        heurs[hidx] = bfs_3d;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;
    }

    // Fullbody Heuristics
    ROS_INFO("Creating Fullbody and Arm Inadmissible heuristics.");

    int goal_base_idx = 0;

    // First goal base pose.
    {
        auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
        bfs_3d_base->setCostPerCell(params.cost_per_cell);
        bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
        if (!bfs_3d_base->init(pspace, grid, 4, goal_base_idx++)) {
            ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
            return false;
        }

        bfs_heurs = { bfs_3d, bfs_3d_base };

        auto retract_arm = std::make_shared<RetractArmHeuristic>();
        if(!retract_arm->init(bfs_3d_base, bfs_3d)){
            ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
            return false;
        }
        // End-Eff only
        auto end_eff_rot = std::make_shared<EndEffHeuristic>();
        end_eff_rot->init(bfs_3d_base, bfs_3d);
        heurs[hidx] = end_eff_rot;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;

        //heurs[hidx] = end_eff_rot;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        //heurs[hidx] = end_eff_rot;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        //heurs[hidx] = end_eff_rot;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        //end_eff_rot = std::make_shared<EndEffHeuristic>();
        //end_eff_rot->init(bfs_3d_base, bfs_3d);
        //heurs[hidx] = end_eff_rot;
        //rep_ids[hidx] = (int) Arm;
        //hidx++;

        // End-eff + Base
        auto inad = std::make_shared<ImprovedEndEffHeuristic>();
        inad->init( bfs_3d_base, bfs_3d, retract_arm );
        heurs[hidx] = inad;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;

        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        //inad = std::make_shared<ImprovedEndEffHeuristic>();
        //inad->init( bfs_3d_base, bfs_3d, retract_arm );
        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Arm;
        //hidx++;

        // Base Only
        int num_rot_heurs = 8;
        for(int i = 0; i < num_rot_heurs; i++){
            auto inad = std::make_shared<BaseRotHeuristic>();
            if (!inad->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
                ROS_ERROR("Could not initialize heuristic.");
                return false;
            }
            heurs[hidx] = inad;
            rep_ids[hidx] = (int) Base;
            hidx++;
        }

    }
    // Second goal base pose
    {
        auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
        bfs_3d_base->setCostPerCell(params.cost_per_cell);
        bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
        if (!bfs_3d_base->init(pspace, grid, 4, goal_base_idx++)) {
            ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
            return false;
        }

        bfs_heurs.push_back(bfs_3d_base);

        auto retract_arm = std::make_shared<RetractArmHeuristic>();
        if(!retract_arm->init(bfs_3d_base, bfs_3d)){
            ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
            return false;
        }
        // End-eff + Base
        auto inad = std::make_shared<ImprovedEndEffHeuristic>();
        inad->init( bfs_3d_base, bfs_3d, retract_arm );
        heurs[hidx] = inad;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;

        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;
        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;
        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;
        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;
        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        //inad = std::make_shared<ImprovedEndEffHeuristic>();
        //inad->init( bfs_3d_base, bfs_3d, retract_arm );
        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Arm;
        //hidx++;

        // Base Only
        int num_rot_heurs = 8;
        for(int i = 0; i < num_rot_heurs; i++){
            auto inad = std::make_shared<BaseRotHeuristic>();
            if (!inad->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
                ROS_ERROR("Could not initialize heuristic.");
                return false;
            }
            heurs[hidx] = inad;
            rep_ids[hidx] = (int) Base;
            hidx++;
        }
    }
    // Third goal base pose
    {
        auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
        bfs_3d_base->setCostPerCell(params.cost_per_cell);
        bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
        if (!bfs_3d_base->init(pspace, grid, 4, goal_base_idx++)) {
            ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
            return false;
        }

        auto retract_arm = std::make_shared<RetractArmHeuristic>();
        if(!retract_arm->init(bfs_3d_base, bfs_3d)){
            ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
            return false;
        }
        // End-eff + Base
        auto inad = std::make_shared<ImprovedEndEffHeuristic>();
        inad->init( bfs_3d_base, bfs_3d, retract_arm );
        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Arm;
        //hidx++;

        // Base Only
        //int num_rot_heurs = 8;
        //for(int i = 0; i < num_rot_heurs; i++){
            //auto inad = std::make_shared<BaseRotHeuristic>();
            //if (!inad->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
                //ROS_ERROR("Could not initialize heuristic.");
                //return false;
            //}
            //heurs[hidx] = inad;
            //rep_ids[hidx] = (int) Base;
            //hidx++;
        //}
    }
    // Fourth goal base pose
    {
        auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
        bfs_3d_base->setCostPerCell(params.cost_per_cell);
        bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
        if (!bfs_3d_base->init(pspace, grid, 4, goal_base_idx++)) {
            ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
            return false;
        }

        auto retract_arm = std::make_shared<RetractArmHeuristic>();
        if(!retract_arm->init(bfs_3d_base, bfs_3d)){
            ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
            return false;
        }
        // End-eff + Base
        auto inad = std::make_shared<ImprovedEndEffHeuristic>();
        inad->init( bfs_3d_base, bfs_3d, retract_arm );
        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Arm;
        //hidx++;

        // Base Only
        //int num_rot_heurs = 8;
        //for(int i = 0; i < num_rot_heurs; i++){
            //auto inad = std::make_shared<BaseRotHeuristic>();
            //if (!inad->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
                //ROS_ERROR("Could not initialize heuristic.");
                //return false;
            //}
            //heurs[hidx] = inad;
            //rep_ids[hidx] = (int) Base;
            //hidx++;
        //}
    }

    // Base heuristics

    assert( hidx == NUM_QUEUES );

    for (auto& entry : heurs) {
        pspace->insertHeuristic(entry.get());
    }
    return true;
}

bool constructHeuristicsMeta(
        std::array< std::shared_ptr<smpl::RobotHeuristic>, NUM_QUEUES >& heurs,
        std::array<int, NUM_QUEUES>& rep_ids,
        std::vector< std::shared_ptr<smpl::RobotHeuristic> >& bfs_heurs,
        smpl::ManipLattice* pspace,
        smpl::OccupancyGrid* grid,
        smpl::KDLRobotModel* rm,
        PlannerConfig& params ){

    SMPL_INFO("Initialize Heuristics");
    const int DefaultCostMultiplier = 1000;


    auto bfs_3d = std::make_shared<smpl::Bfs3DHeuristic>();
    bfs_3d->setCostPerCell(params.cost_per_cell);
    bfs_3d->setInflationRadius(params.inflation_radius_3d);
    if (!bfs_3d->init(pspace, grid)) {
        ROS_ERROR("Could not initialize Bfs3Dheuristic.");
        return false;
    }

    int hidx = 0;
    ROS_INFO("Creating Fullbody Anchor Heuristic");
    {
        // This won't be admissible
        //auto anchor = std::make_unique<AnchorHeuristic>();
        //anchor->init( bfs_3d_base, bfs_3d );
        heurs[hidx] = bfs_3d;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;
    }

    // Fullbody Heuristics
    ROS_INFO("Creating Fullbody and Arm Inadmissible heuristics.");

    int goal_base_idx = 0;


    // First goal base pose.
    {
        auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
        bfs_3d_base->setCostPerCell(params.cost_per_cell);
        bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
        if (!bfs_3d_base->init(pspace, grid, 4, goal_base_idx++)) {
            ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
            return false;
        }

        bfs_heurs = { bfs_3d, bfs_3d_base };

        auto retract_arm = std::make_shared<RetractArmHeuristic>();
        if(!retract_arm->init(bfs_3d_base, bfs_3d)){
            ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
            return false;
        }
        // End-Eff only
        auto end_eff_rot = std::make_shared<EndEffHeuristic>();
        end_eff_rot->init(bfs_3d_base, bfs_3d);
        heurs[hidx] = end_eff_rot;
        rep_ids[hidx] = (int) Fullbody;//Arm;
        hidx++;

        //heurs[hidx] = end_eff_rot;
        //rep_ids[hidx] = (int) Base;//Fullbody;
        //hidx++;

        // End-eff + Base
        auto inad = std::make_shared<ImprovedEndEffHeuristic>();
        inad->init( bfs_3d_base, bfs_3d, retract_arm );
        heurs[hidx] = inad;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;

        int num_rot_heurs = 1;
        for(int i = 0; i < num_rot_heurs; i++){
            auto inad = std::make_shared<BaseRotHeuristic>();
            if (!inad->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
                ROS_ERROR("Could not initialize heuristic.");
                return false;
            }
            inad->orientation_coeff = 0.0;
            heurs[hidx] = inad;
            rep_ids[hidx] = (int) Base;
            hidx++;
        }

    }

    for(int i = 0; i < 5; i++)
    {
        auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
        bfs_3d_base->setCostPerCell(params.cost_per_cell);
        bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
        if (!bfs_3d_base->init(pspace, grid, 4, goal_base_idx++)) {
            ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
            return false;
        }

        bfs_heurs.push_back(bfs_3d_base);

        auto retract_arm = std::make_shared<RetractArmHeuristic>();
        if(!retract_arm->init(bfs_3d_base, bfs_3d)){
            ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
            return false;
        }

        auto inad = std::make_shared<ImprovedEndEffHeuristic>();
        inad->init( bfs_3d_base, bfs_3d, retract_arm );
        heurs[hidx] = inad;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;

        // Base Only
        int num_rot_heurs = 1;
        for(int i = 0; i < num_rot_heurs; i++){
            auto inad = std::make_shared<BaseRotHeuristic>();
            if (!inad->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
                ROS_ERROR("Could not initialize heuristic.");
                return false;
            }
            inad->orientation_coeff = 0.0;
            heurs[hidx] = inad;
            rep_ids[hidx] = (int) Base;
            hidx++;
        }
    }
    // Base heuristics

    assert( hidx == NUM_QUEUES );

    for (auto& entry : heurs) {
        pspace->insertHeuristic(entry.get());
    }
    return true;
}

bool constructHeuristicsFullbody(
        std::array< std::shared_ptr<smpl::RobotHeuristic>, NUM_QUEUES >& heurs,
        std::array<int, NUM_QUEUES>& rep_ids,
        std::vector< std::shared_ptr<smpl::RobotHeuristic> >& bfs_heurs,
        smpl::ManipLattice* pspace,
        smpl::OccupancyGrid* grid,
        smpl::KDLRobotModel* rm,
        PlannerConfig& params ){

    SMPL_INFO("Initialize Heuristics");
    const int DefaultCostMultiplier = 1000;


    auto bfs_3d = std::make_shared<smpl::Bfs3DHeuristic>();
    bfs_3d->setCostPerCell(params.cost_per_cell);
    bfs_3d->setInflationRadius(params.inflation_radius_3d);
    if (!bfs_3d->init(pspace, grid)) {
        ROS_ERROR("Could not initialize Bfs3Dheuristic.");
        return false;
    }

    int hidx = 0;
    ROS_INFO("Creating Fullbody Anchor Heuristic");
    {
        // This won't be admissible
        //auto anchor = std::make_unique<AnchorHeuristic>();
        //anchor->init( bfs_3d_base, bfs_3d );
        heurs[hidx] = bfs_3d;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;
    }

    // Fullbody Heuristics
    ROS_INFO("Creating Fullbody and Arm Inadmissible heuristics.");

    int goal_base_idx = 0;

    // First goal base pose.
    {
        auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
        bfs_3d_base->setCostPerCell(params.cost_per_cell);
        bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
        if (!bfs_3d_base->init(pspace, grid, 4, goal_base_idx++)) {
            ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
            return false;
        }

        bfs_heurs = { bfs_3d, bfs_3d_base };

        auto retract_arm = std::make_shared<RetractArmHeuristic>();
        if(!retract_arm->init(bfs_3d_base, bfs_3d)){
            ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
            return false;
        }
        // End-Eff only
        auto end_eff_rot = std::make_shared<EndEffHeuristic>();
        end_eff_rot->init(bfs_3d_base, bfs_3d);
        heurs[hidx] = end_eff_rot;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;

        // End-eff + Base
        //auto inad = std::make_shared<ImprovedEndEffHeuristic>();
        //inad->init( bfs_3d_base, bfs_3d, retract_arm );
        //heurs[hidx] = inad;
        //rep_ids[hidx] = (int) Fullbody;
        //hidx++;

        int num_rot_heurs = 1;
        for(int i = 0; i < num_rot_heurs; i++){
            auto inad = std::make_shared<BaseRotHeuristic>();
            if (!inad->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
                ROS_ERROR("Could not initialize heuristic.");
                return false;
            }
            inad->orientation_coeff = 0.0;
            heurs[hidx] = inad;
            rep_ids[hidx] = (int) Fullbody;
            hidx++;
        }

    }
    /*
    {
        auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
        bfs_3d_base->setCostPerCell(params.cost_per_cell);
        bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
        if (!bfs_3d_base->init(pspace, grid, 4, goal_base_idx++)) {
            ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
            return false;
        }

        bfs_heurs.push_back(bfs_3d_base);

        auto retract_arm = std::make_shared<RetractArmHeuristic>();
        if(!retract_arm->init(bfs_3d_base, bfs_3d)){
            ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
            return false;
        }

        // Base Only
        int num_rot_heurs = 1;
        for(int i = 0; i < num_rot_heurs; i++){
            auto inad = std::make_shared<BaseRotHeuristic>();
            if (!inad->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
                ROS_ERROR("Could not initialize heuristic.");
                return false;
            }
            inad->orientation_coeff = 0.0;
            heurs[hidx] = inad;
            rep_ids[hidx] = (int) Base;
            hidx++;
        }
    }
    */

    // Base heuristics

    assert( hidx == NUM_QUEUES );

    for (auto& entry : heurs) {
        pspace->insertHeuristic(entry.get());
    }
    return true;
}

bool constructHeuristicsMultiRep(
        std::array< std::shared_ptr<smpl::RobotHeuristic>, NUM_QUEUES >& heurs,
        std::array<int, NUM_QUEUES>& rep_ids,
        std::vector< std::shared_ptr<smpl::RobotHeuristic> >& bfs_heurs,
        smpl::ManipLattice* pspace,
        smpl::OccupancyGrid* grid,
        smpl::KDLRobotModel* rm,
        PlannerConfig& params ){

    SMPL_INFO("Initialize Heuristics");
    const int DefaultCostMultiplier = 1000;


    auto bfs_3d = std::make_shared<smpl::Bfs3DHeuristic>();
    bfs_3d->setCostPerCell(params.cost_per_cell);
    bfs_3d->setInflationRadius(params.inflation_radius_3d);
    if (!bfs_3d->init(pspace, grid)) {
        ROS_ERROR("Could not initialize Bfs3Dheuristic.");
        return false;
    }

    int hidx = 0;
    ROS_INFO("Creating Fullbody Anchor Heuristic");
    {
        // This won't be admissible
        //auto anchor = std::make_unique<AnchorHeuristic>();
        //anchor->init( bfs_3d_base, bfs_3d );
        heurs[hidx] = bfs_3d;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;
    }

    // Fullbody Heuristics
    ROS_INFO("Creating Fullbody and Arm Inadmissible heuristics.");

    int goal_base_idx = 0;

    // First goal base pose.
    {
        auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
        bfs_3d_base->setCostPerCell(params.cost_per_cell);
        bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
        if (!bfs_3d_base->init(pspace, grid, 4, goal_base_idx++)) {
            ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
            return false;
        }

        bfs_heurs = { bfs_3d, bfs_3d_base };

        auto retract_arm = std::make_shared<RetractArmHeuristic>();
        if(!retract_arm->init(bfs_3d_base, bfs_3d)){
            ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
            return false;
        }
        // End-Eff only
        auto end_eff_rot = std::make_shared<EndEffHeuristic>();
        end_eff_rot->init(bfs_3d_base, bfs_3d);
        heurs[hidx] = end_eff_rot;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;

        // End-eff + Base
        auto inad = std::make_shared<ImprovedEndEffHeuristic>();
        inad->init( bfs_3d_base, bfs_3d, retract_arm );
        inad->rot_coeff = 0.0;
        heurs[hidx] = inad;
        rep_ids[hidx] = (int) Fullbody;
        hidx++;

        int num_rot_heurs = 1;
        for(int i = 0; i < num_rot_heurs; i++){
            auto inad = std::make_shared<BaseRotHeuristic>();
            if (!inad->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
                ROS_ERROR("Could not initialize heuristic.");
                return false;
            }
            inad->orientation_coeff = 0.0;
            heurs[hidx] = inad;
            rep_ids[hidx] = (int) Base;
            hidx++;
        }

    }
    /*
    {
        auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
        bfs_3d_base->setCostPerCell(params.cost_per_cell);
        bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
        if (!bfs_3d_base->init(pspace, grid, 4, goal_base_idx++)) {
            ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
            return false;
        }

        bfs_heurs.push_back(bfs_3d_base);

        auto retract_arm = std::make_shared<RetractArmHeuristic>();
        if(!retract_arm->init(bfs_3d_base, bfs_3d)){
            ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
            return false;
        }

        // Base Only
        int num_rot_heurs = 1;
        for(int i = 0; i < num_rot_heurs; i++){
            auto inad = std::make_shared<BaseRotHeuristic>();
            if (!inad->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
                ROS_ERROR("Could not initialize heuristic.");
                return false;
            }
            inad->orientation_coeff = 0.0;
            heurs[hidx] = inad;
            rep_ids[hidx] = (int) Base;
            hidx++;
        }
    }
    */

    // Base heuristics

    assert( hidx == NUM_QUEUES );

    for (auto& entry : heurs) {
        pspace->insertHeuristic(entry.get());
    }
    return true;
}
