#pragma once

#include "planner.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>

namespace ompl::geometric { class SimpleSetup; }

namespace motion_planning_examples
{

struct RRTStarSettings
{
    double solveTime{1.2};
    double range{0.30};
    double goalBias{0.05};
    double rewireFactor{1.10};
    int pathInterpolationPoints{220};
};

class OMPLPlanner : public Planner
{
public:
    explicit OMPLPlanner(std::shared_ptr<ompl::base::StateSpace> space);

    void setStateValidityChecker(const std::function<bool(const ompl::base::State *)> &checker);
    void configureRRTStar(const RRTStarSettings &settings);

    void setStartGoal(double startTheta1, double startTheta2, double goalTheta1, double goalTheta2) override;
    bool solve(double solveTimeSeconds) override;
    void simplifyPath(double maxTime = 1.0) override;

    [[nodiscard]] std::vector<std::pair<double, double>> getPathAngles() const override;
    [[nodiscard]] double getPathLength() const override;

private:
    std::shared_ptr<ompl::base::StateSpace> space_;
    std::shared_ptr<ompl::geometric::SimpleSetup> simpleSetup_;
    int interpolationPoints_{100};
};

}  // namespace motion_planning_examples
