#pragma once

#include "manifold_types.hpp"

namespace motion_planning_examples
{

class Planner
{
public:
    virtual ~Planner() = default;

    // Set start and goal directly on the joint manifold representation.
    virtual void setStartGoal(const JointManifoldState &start, const JointManifoldState &goal) = 0;

    // Solves the problem within the defined configuration timeout
    virtual bool solve(double solveTimeSeconds) = 0;

    // Retrieve the solution as a path of manifold states (no scalar angle wrapping required).
    [[nodiscard]] virtual ManifoldPath getPathManifoldStates() const = 0;
    
    [[nodiscard]] virtual double getPathLength() const = 0;
};

}  // namespace motion_planning_examples