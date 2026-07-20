#pragma once

#include "planner.hpp"
#include "robot_mechanism.hpp"

#include <cstddef>
#include <functional>
#include <memory>

#include <ompl/base/ScopedState.h>

namespace ompl::geometric
{
class SimpleSetup;
}

namespace motion_planning_examples
{

struct RRTStarSettings
{
  double solveTime{ 1.2 };
  double range{ 0.30 };
  double goalBias{ 0.05 };
  double rewireFactor{ 1.10 };
  int pathInterpolationPoints{ 220 };
};

class OMPLPlanner : public Planner
{
public:
  explicit OMPLPlanner(std::shared_ptr<RobotMechanism> arm);

  void setStateValidityChecker(const std::function<bool(const JointManifoldState&)>& checker);
  void configureRRTStar(const RRTStarSettings& settings);

  void setStartGoal(const JointManifoldState& start, const JointManifoldState& goal) override;

  bool solve(double solveTimeSeconds) override;

  [[nodiscard]] ManifoldPath getPathManifoldStates() const override;
  [[nodiscard]] double getPathLength() const override;

private:
  [[nodiscard]] std::shared_ptr<ompl::base::StateSpace> createStateSpace() const;
  [[nodiscard]] JointManifoldState manifoldStateFromOMPL(const ompl::base::State* state) const;
  void setOMPLStateFromManifold(const JointManifoldState& state, ompl::base::State* outState) const;

  std::shared_ptr<RobotMechanism> arm_;
  std::shared_ptr<ompl::base::StateSpace> stateSpace_;
  std::shared_ptr<ompl::geometric::SimpleSetup> simpleSetup_;
  int interpolationPoints_{ 100 };
};

}  // namespace motion_planning_examples
