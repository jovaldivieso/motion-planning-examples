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

    void setStartGoal(const JointManifoldState &start, const JointManifoldState &goal) override;
    bool solve(double solveTimeSeconds) override;

    [[nodiscard]] ManifoldPath getPathManifoldStates() const override;
    [[nodiscard]] double getPathLength() const override;

private:
    std::shared_ptr<RobotMechanism> arm_;
    std::shared_ptr<FCLCollisionChecker> checker_;
    
    int numWaypoints_;

    std::vector<double> startTaskSpace_;
    std::vector<double> goalTaskSpace_;
    std::size_t jointCount_{0};
    std::size_t taskSpaceDimension_{0};
    TaskSpaceType taskSpaceType_{TaskSpaceType::Euclidean2D};

    JointManifoldState startState_;
    JointManifoldState goalState_;

    std::vector<double> v1Path_;
    std::vector<double> v2Path_;
    std::vector<double> v3Path_;
};

}  // namespace motion_planning_examples
