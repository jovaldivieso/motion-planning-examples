#pragma once

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "two_dof_planar_arm.hpp"
#include "fcl_collision_checker.hpp"

namespace motion_planning_examples
{

class CeresPlanner
{
public:
    CeresPlanner(std::shared_ptr<TwoDOFPlanarArm> arm, 
                 std::shared_ptr<FCLCollisionChecker> checker,
                 double link1Length, 
                 double link2Length, 
                 int numWaypoints = 100);

    // Formulate start and goal purely in Cartesian Workspace
    // We pass elbowUp to select the correct IK branch that matches our configuration
    void setStartGoalWorkspace(double startX, double startY, double goalX, double goalY, bool elbowUp);

    // Runs the trajectory optimization and strictly checks the result for collisions
    bool solve();

    // Returns the C-Space path for execution and visualization
    std::vector<std::pair<double, double>> getPathAngles() const;

private:
    // Trigonometry-free SO(2) algebraic IK to seed the initial guess
    bool computeAlgebraicIK(double px, double py, bool elbow_up,
                            double& v1x, double& v1y, 
                            double& v2x, double& v2y) const;

    std::shared_ptr<TwoDOFPlanarArm> arm_;
    std::shared_ptr<FCLCollisionChecker> checker_;
    
    double l1_;
    double l2_;
    int numWaypoints_;

    double startX_{0.0};
    double startY_{0.0};
    double goalX_{0.0};
    double goalY_{0.0};
    bool elbowUp_{true};

    // Continuous block of variables for Ceres where each is a 2D vector [x, y] on S^1
    std::vector<double> v1Path_;
    std::vector<double> v2Path_;
};

}  // namespace motion_planning_examples
