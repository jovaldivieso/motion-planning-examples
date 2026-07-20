#include "ompl_planner.hpp"

#include <cmath>
#include <stdexcept>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace motion_planning_examples
{

std::shared_ptr<ob::StateSpace> OMPLPlanner::createStateSpace() const
{
    auto compound = std::make_shared<ob::CompoundStateSpace>();
    for (std::size_t i = 0; i < arm_->getJointCount(); ++i)
    {
        compound->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
    }
    compound->lock();
    return compound;
}

JointManifoldState OMPLPlanner::manifoldStateFromOMPL(const ob::State *state) const
{
    const auto *compound = state->as<ob::CompoundStateSpace::StateType>();
    const unsigned int subspaceCount = stateSpace_->as<ob::CompoundStateSpace>()->getSubspaceCount();

    JointManifoldState manifold;
    manifold.reserve(2 * subspaceCount);
    for (unsigned int i = 0; i < subspaceCount; ++i)
    {
        const double theta = compound->as<ob::SO2StateSpace::StateType>(i)->value;
        const auto joint = createFromAngleSO2(theta);
        manifold.push_back(joint[0]);
        manifold.push_back(joint[1]);
    }
    return manifold;
}

void OMPLPlanner::setOMPLStateFromManifold(const JointManifoldState &state, ob::State *outState) const
{
    auto *compound = outState->as<ob::CompoundStateSpace::StateType>();
    const unsigned int subspaceCount = stateSpace_->as<ob::CompoundStateSpace>()->getSubspaceCount();
    if (state.size() < 2 * subspaceCount)
    {
        throw std::invalid_argument("Manifold state dimension does not match OMPL state space");
    }

    for (unsigned int i = 0; i < subspaceCount; ++i)
    {
        const double c = state[2 * i];
        const double s = state[2 * i + 1];
        compound->as<ob::SO2StateSpace::StateType>(i)->value = convertToAngleSO2({c, s});
    }
}

OMPLPlanner::OMPLPlanner(std::shared_ptr<RobotMechanism> arm)
    : arm_(std::move(arm))
{
    stateSpace_ = createStateSpace();
    simpleSetup_ = std::make_shared<og::SimpleSetup>(stateSpace_);
}

void OMPLPlanner::setStateValidityChecker(const std::function<bool(const JointManifoldState &)> &checker)
{
    simpleSetup_->setStateValidityChecker([this, checker](const ob::State *state) {
        return checker(manifoldStateFromOMPL(state));
    });
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
    ob::ScopedState<> startState(stateSpace_);
    setOMPLStateFromManifold(start, startState.get());

    ob::ScopedState<> goalState(stateSpace_);
    setOMPLStateFromManifold(goal, goalState.get());

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
        waypoints.push_back(manifoldStateFromOMPL(path.getState(i)));
    }

    return waypoints;
}

double OMPLPlanner::getPathLength() const
{
    if (!simpleSetup_->haveSolutionPath()) return 0.0;
    return simpleSetup_->getSolutionPath().length();
}

}  // namespace motion_planning_examples
