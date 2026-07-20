#pragma once

#include "manifolds.hpp"

namespace motion_planning_examples
{

class Planner
{
public:
  virtual ~Planner() = default;

  virtual void setStartGoal(const JointManifoldState& start, const JointManifoldState& goal) = 0;
  virtual bool solve(double solveTimeSeconds) = 0;

  [[nodiscard]] virtual ManifoldPath getPathManifoldStates() const = 0;
  [[nodiscard]] virtual double getPathLength() const = 0;
};

}  // namespace motion_planning_examples
