#include "ompl_planner.hpp"

#include <cmath>
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
    if (start.size() < 2 || goal.size() < 2)
    {
        throw std::invalid_argument("OMPLPlanner expects at least 2 manifold joints for start and goal");
    }

    const double startTheta1 = std::atan2(start[0][1], start[0][0]);
    const double startTheta2 = std::atan2(start[1][1], start[1][0]);
    const double goalTheta1 = std::atan2(goal[0][1], goal[0][0]);
    const double goalTheta2 = std::atan2(goal[1][1], goal[1][0]);

    ob::ScopedState<> startState(space_);
    startState[0] = startTheta1;
    startState[1] = startTheta2;

    ob::ScopedState<> goalState(space_);
    goalState[0] = goalTheta1;
    goalState[1] = goalTheta2;

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
        const auto *state = path.getState(i)->as<ob::CompoundStateSpace::StateType>();
        const double theta1 = state->as<ob::SO2StateSpace::StateType>(0)->value;
        const double theta2 = state->as<ob::SO2StateSpace::StateType>(1)->value;
        waypoints.push_back({
            JointS1{std::cos(theta1), std::sin(theta1)},
            JointS1{std::cos(theta2), std::sin(theta2)}
        });
    }

    return waypoints;
}

double OMPLPlanner::getPathLength() const
{
    if (!simpleSetup_->haveSolutionPath()) return 0.0;
    return simpleSetup_->getSolutionPath().length();
}

}  // namespace motion_planning_examples