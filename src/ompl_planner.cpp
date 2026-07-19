#include "ompl_planner.hpp"

#include <stdexcept>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace motion_planning_examples
{

OMPLPlanner::OMPLPlanner(std::shared_ptr<RobotMechanism> arm)
    : arm_(std::move(arm))
{
    simpleSetup_ = std::make_shared<og::SimpleSetup>(arm_->getStateSpace());
}

void OMPLPlanner::setStateValidityChecker(const std::function<bool(const ob::State *)> &checker)
{
    simpleSetup_->setStateValidityChecker(checker);
}

void OMPLPlanner::configureRRTStar(const RRTStarSettings &settings)
{
    auto planner = std::make_shared<og::RRTstar>(simpleSetup_->getSpaceInformation());
    planner->setRange(settings.range);
    planner->setGoalBias(settings.goalBias);
    planner->setRewireFactor(settings.rewireFactor);
    simpleSetup_->setPlanner(planner);
    interpolationPoints_ = settings.pathInterpolationPoints;
}

void OMPLPlanner::setStartGoal(const JointManifoldState &start, const JointManifoldState &goal)
{
    ob::ScopedState<> startState(arm_->getStateSpace());
    arm_->setOMPLState(start, startState.get());

    ob::ScopedState<> goalState(arm_->getStateSpace());
    arm_->setOMPLState(goal, goalState.get());

    simpleSetup_->setStartAndGoalStates(startState, goalState);
}

bool OMPLPlanner::solve(double solveTimeSeconds)
{
    bool solved = static_cast<bool>(simpleSetup_->solve(solveTimeSeconds));
    if (solved)
    {
        // OMPL specific smoothing executed internally
        simpleSetup_->simplifySolution(1.5);
    }
    return solved;
}

ManifoldPath OMPLPlanner::getPathManifoldStates() const
{
    if (!simpleSetup_->haveSolutionPath()) throw std::runtime_error("No solution path available");

    og::PathGeometric path = simpleSetup_->getSolutionPath();
    path.interpolate(static_cast<unsigned int>(interpolationPoints_));

    ManifoldPath waypoints;
    waypoints.reserve(path.getStateCount());

    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        waypoints.push_back(arm_->getManifoldState(path.getState(i)));
    }

    return waypoints;
}

double OMPLPlanner::getPathLength() const
{
    if (!simpleSetup_->haveSolutionPath()) return 0.0;
    return simpleSetup_->getSolutionPath().length();
}

}  // namespace motion_planning_examples
