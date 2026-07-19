#include "ompl_planner.hpp"

#include <stdexcept>

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace motion_planning_examples
{

OMPLPlanner::OMPLPlanner(std::shared_ptr<ob::StateSpace> space)
    : space_(std::move(space))
{
    simpleSetup_ = std::make_shared<og::SimpleSetup>(space_);
}

void OMPLPlanner::setStateValidityChecker(const std::function<bool(const ob::State *)> &checker)
{
    simpleSetup_->setStateValidityChecker(checker);
}

void OMPLPlanner::setStartGoal(double startTheta1, double startTheta2, double goalTheta1, double goalTheta2)
{
    ob::ScopedState<> start(space_);
    start[0] = startTheta1;
    start[1] = startTheta2;

    ob::ScopedState<> goal(space_);
    goal[0] = goalTheta1;
    goal[1] = goalTheta2;

    simpleSetup_->setStartAndGoalStates(start, goal);
}

void OMPLPlanner::configureRRTStar(const RRTStarSettings &settings)
{
    auto planner = std::make_shared<og::RRTstar>(simpleSetup_->getSpaceInformation());
    planner->setRange(settings.range);
    planner->setGoalBias(settings.goalBias);
    planner->setRewireFactor(settings.rewireFactor);
    simpleSetup_->setPlanner(planner);
}

bool OMPLPlanner::solve(double solveTimeSeconds)
{
    return static_cast<bool>(simpleSetup_->solve(solveTimeSeconds));
}

void OMPLPlanner::simplifyPath(double maxTime)
{
    if (simpleSetup_->haveSolutionPath())
    {
        simpleSetup_->simplifySolution(maxTime);
    }
}

std::vector<std::pair<double, double>> OMPLPlanner::getInterpolatedPath(int interpolationPoints) const
{
    if (!simpleSetup_->haveSolutionPath())
    {
        throw std::runtime_error("No solution path available");
    }

    og::PathGeometric path = simpleSetup_->getSolutionPath();
    path.interpolate(static_cast<unsigned int>(interpolationPoints));

    std::vector<std::pair<double, double>> waypoints;
    waypoints.reserve(path.getStateCount());

    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        const auto *state = path.getState(i)->as<ob::CompoundStateSpace::StateType>();
        const double theta1 = state->as<ob::SO2StateSpace::StateType>(0)->value;
        const double theta2 = state->as<ob::SO2StateSpace::StateType>(1)->value;
        waypoints.emplace_back(theta1, theta2);
    }

    return waypoints;
}

double OMPLPlanner::getPathLength() const
{
    if (!simpleSetup_->haveSolutionPath())
    {
        return 0.0;
    }
    return simpleSetup_->getSolutionPath().length();
}

std::size_t OMPLPlanner::getPathStateCount() const
{
    if (!simpleSetup_->haveSolutionPath())
    {
        return 0;
    }
    return simpleSetup_->getSolutionPath().getStateCount();
}

std::shared_ptr<ob::StateSpace> OMPLPlanner::getStateSpace() const
{
    return space_;
}

}  // namespace motion_planning_examples
