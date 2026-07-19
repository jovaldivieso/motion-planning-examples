#pragma once

#include "planner.hpp"
#include "robot_mechanism.hpp"
#include "fcl_collision_checker.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace motion_planning_examples
{

class CeresPlanner : public Planner
{
public:
    CeresPlanner(std::shared_ptr<RobotMechanism> arm, 
                 std::shared_ptr<FCLCollisionChecker> checker,
                 int numWaypoints = 100);

    // Ceres internally converts C-Space goals to Workspace tracking lines
    void setStartGoal(double startTheta1, double startTheta2, double goalTheta1, double goalTheta2) override;

    bool solve(double solveTimeSeconds) override;

    [[nodiscard]] std::vector<std::pair<double, double>> getPathAngles() const override;
    [[nodiscard]] double getPathLength() const override;

private:
    std::shared_ptr<RobotMechanism> arm_;
    std::shared_ptr<FCLCollisionChecker> checker_;
    
    int numWaypoints_;

    double startX_{0.0};
    double startY_{0.0};
    double goalX_{0.0};
    double goalY_{0.0};
    bool elbowUp_{true};

    std::vector<double> v1Path_;
    std::vector<double> v2Path_;
};

}  // namespace motion_planning_examples
