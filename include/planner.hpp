#pragma once

#include <vector>
#include <utility>

namespace motion_planning_examples
{

class Planner
{
public:
    virtual ~Planner() = default;

    // Both planners will accept goals cleanly in configuration space
    virtual void setStartGoal(double startTheta1, double startTheta2, double goalTheta1, double goalTheta2) = 0;

    // Solves the problem within the defined configuration timeout
    virtual bool solve(double solveTimeSeconds) = 0;

    // Provides an optional hook for post-processing routines (e.g., OMPL smoothing)
    virtual void simplifyPath(double /*maxTime*/ = 1.0) {}

    // Retrieves the final unified format output
    [[nodiscard]] virtual std::vector<std::pair<double, double>> getPathAngles() const = 0;
    
    [[nodiscard]] virtual double getPathLength() const = 0;
};

}  // namespace motion_planning_examples
