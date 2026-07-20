#pragma once

#include "planner.hpp"
#include "robot_mechanism.hpp"
#include "fcl_collision_checker.hpp"

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

namespace ceres
{
class Problem;
}

namespace motion_planning_examples
{

struct CeresPlannerOptions
{
    double lineConstraintWeight{20.0};
    double smoothnessWeight{1.0};
    int maxNumIterations{100};
};

class CeresPlanner : public Planner
{
public:
    CeresPlanner(std::shared_ptr<RobotMechanism> arm, 
                 std::shared_ptr<FCLCollisionChecker> checker,
                 int numWaypoints = 100,
                 const CeresPlannerOptions &options = {});

    void setStartGoal(const JointManifoldState &start, const JointManifoldState &goal) override;
    bool solve(double solveTimeSeconds) override;

    [[nodiscard]] ManifoldPath getPathManifoldStates() const override;
    [[nodiscard]] double getPathLength() const override;

private:
    using SO2Coordinates = std::array<double, 2>;
    using JointPath = std::vector<SO2Coordinates>;

    std::shared_ptr<RobotMechanism> arm_;
    std::shared_ptr<FCLCollisionChecker> checker_;
    
    int numWaypoints_;

    std::vector<double> startTaskSpace_;
    std::vector<double> goalTaskSpace_;
    TaskSpaceCoordinates startTaskSpaceCoordinates_{};
    TaskSpaceCoordinates goalTaskSpaceCoordinates_{};
    std::size_t jointCount_{0};
    std::size_t taskSpaceDimension_{0};
    TaskSpaceType taskSpaceType_{TaskSpaceType::Euclidean2D};
    CeresPlannerOptions options_{};

    JointManifoldState startState_;
    JointManifoldState goalState_;

    std::vector<JointPath> jointPaths_;

    bool computeTaskSpaceDirection(std::vector<double> &direction) const;
    bool initializeWaypoints();
    bool optimizePath(const std::vector<double> &direction, double solveTimeSeconds);
    bool runSolver(ceres::Problem &problem, double solveTimeSeconds) const;
    bool validatePath() const;
    void assignWaypointFromState(int waypointIndex, const JointManifoldState &state);
    [[nodiscard]] double* jointPathBlock(std::size_t jointIndex, int waypointIndex);
    [[nodiscard]] JointManifoldState buildFlatState(int waypointIndex) const;
};

}  // namespace motion_planning_examples
