#include "ceres_planner.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <ceres/ceres.h>

namespace motion_planning_examples
{

namespace
{

std::array<double, 16> buildProjectionMatrix4D(const std::vector<double> &dir)
{
    std::array<double, 16> p{};
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            const double identity = (row == col) ? 1.0 : 0.0;
            p[4 * row + col] = identity - dir[row] * dir[col];
        }
    }
    return p;
}

}  // namespace

struct StraightLineCost2D
{
    StraightLineCost2D(double l1, double l2, double tx, double ty, double p00, double p01, double p11, double weight)
        : l1_(l1), l2_(l2), tx_(tx), ty_(ty), p00_(p00), p01_(p01), p11_(p11), w_(weight) {}

    template <typename T>
    bool operator()(const T* const v1, const T* const v2, T* residual) const
    {
        T v12_x = v1[0] * v2[0] - v1[1] * v2[1];
        T v12_y = v1[1] * v2[0] + v1[0] * v2[1];
        
        T ee_x = T(l1_) * v1[0] + T(l2_) * v12_x;
        T ee_y = T(l1_) * v1[1] + T(l2_) * v12_y;
        
        T dx = ee_x - T(tx_);
        T dy = ee_y - T(ty_);

        residual[0] = T(w_) * (T(p00_) * dx + T(p01_) * dy);
        residual[1] = T(w_) * (T(p01_) * dx + T(p11_) * dy);
        return true;
    }

private:
    double l1_, l2_, tx_, ty_, p00_, p01_, p11_, w_;
};

struct StraightLineCost4D
{
        StraightLineCost4D(double l1, double l2, double l3,
                                             double anchorX, double anchorY, double anchorC, double anchorS,
                                             const std::array<double, 16> &projection,
                       double weight)
                : l1_(l1), l2_(l2), l3_(l3),
                    anchorX_(anchorX), anchorY_(anchorY), anchorC_(anchorC), anchorS_(anchorS),
                    w_(weight), projection_(projection) {}

    template <typename T>
    bool operator()(const T* const v1, const T* const v2, const T* const v3, T* residual) const
    {
        T c1 = v1[0];
        T s1 = v1[1];
        T c2 = v2[0];
        T s2 = v2[1];
        T c3 = v3[0];
        T s3 = v3[1];

        T c12 = c1 * c2 - s1 * s2;
        T s12 = s1 * c2 + c1 * s2;
        T c123 = c12 * c3 - s12 * s3;
        T s123 = s12 * c3 + c12 * s3;

        T ee_x = T(l1_) * c1 + T(l2_) * c12 + T(l3_) * c123;
        T ee_y = T(l1_) * s1 + T(l2_) * s12 + T(l3_) * s123;
        const T diff[4] = {
            ee_x - T(anchorX_),
            ee_y - T(anchorY_),
            c123 - T(anchorC_),
            s123 - T(anchorS_),
        };

        for (int row = 0; row < 4; ++row)
        {
            residual[row] = T(0.0);
            for (int col = 0; col < 4; ++col)
            {
                residual[row] += T(projection_[4 * row + col]) * diff[col];
            }
            residual[row] *= T(w_);
        }
        return true;
    }

private:
    double l1_, l2_, l3_, anchorX_, anchorY_, anchorC_, anchorS_, w_;
    std::array<double, 16> projection_;
};

struct SmoothnessCost
{
    explicit SmoothnessCost(double weight) : w_(weight) {}

    template <typename T>
    bool operator()(const T* const v_curr, const T* const v_next, T* residual) const
    {
        residual[0] = T(w_) * (v_next[0] - v_curr[0]);
        residual[1] = T(w_) * (v_next[1] - v_curr[1]);
        return true;
    }
private:
    double w_;
};

CeresPlanner::CeresPlanner(std::shared_ptr<RobotMechanism> arm, 
                           std::shared_ptr<FCLCollisionChecker> checker,
               int numWaypoints,
               const CeresPlannerOptions &options)
    : arm_(std::move(arm)), checker_(std::move(checker)), 
            numWaypoints_(numWaypoints),
        jointCount_(arm_->getJointCount()),
        options_(options)
{
        if (numWaypoints_ < 2) throw std::invalid_argument("At least 2 waypoints required.");
        if (jointCount_ < 2 || jointCount_ > 3) throw std::invalid_argument("CeresPlanner currently supports 2 or 3 planar joints.");
    if (options_.lineConstraintWeight <= 0.0) throw std::invalid_argument("lineConstraintWeight must be positive.");
    if (options_.smoothnessWeight <= 0.0) throw std::invalid_argument("smoothnessWeight must be positive.");
    if (options_.maxNumIterations <= 0) throw std::invalid_argument("maxNumIterations must be positive.");
    jointPaths_.resize(jointCount_);
    for (auto &jointPath : jointPaths_)
    {
        jointPath.resize(numWaypoints_, SO2Coordinates{0.0, 0.0});
    }
}

void CeresPlanner::setStartGoal(const JointManifoldState &start, const JointManifoldState &goal)
{
    startState_ = start;
    goalState_ = goal;

    startTaskSpaceCoordinates_ = arm_->computeTaskSpaceCoordinates(startState_);
    goalTaskSpaceCoordinates_ = arm_->computeTaskSpaceCoordinates(goalState_);
    startTaskSpace_ = flattenTaskSpaceCoordinates(startTaskSpaceCoordinates_);
    goalTaskSpace_ = flattenTaskSpaceCoordinates(goalTaskSpaceCoordinates_);

    taskSpaceType_ = getTaskSpaceType(startTaskSpaceCoordinates_);
    if (taskSpaceType_ != getTaskSpaceType(goalTaskSpaceCoordinates_))
    {
        throw std::invalid_argument("Start and goal task-space types must match.");
    }

    taskSpaceDimension_ = getTaskSpaceCoordinateCount(taskSpaceType_);
    if (taskSpaceType_ != TaskSpaceType::Euclidean2D && taskSpaceType_ != TaskSpaceType::SE2)
    {
        throw std::invalid_argument("CeresPlanner currently supports 2D Euclidean or SE(2) task spaces.");
    }
}

bool CeresPlanner::solve(double solveTimeSeconds)
{
    std::vector<double> direction;
    if (!computeTaskSpaceDirection(direction)) return false;
    if (!initializeWaypoints()) return false;
    if (!optimizePath(direction, solveTimeSeconds)) return false;
    return validatePath();
}

bool CeresPlanner::computeTaskSpaceDirection(std::vector<double> &direction) const
{
    direction.assign(taskSpaceDimension_, 0.0);
    for (std::size_t i = 0; i < taskSpaceDimension_; ++i)
    {
        direction[i] = goalTaskSpace_[i] - startTaskSpace_[i];
    }

    double len = 0.0;
    for (double value : direction) len += value * value;
    len = std::sqrt(len);
    if (len < 1e-6) return false;

    for (double &value : direction) value /= len;
    return true;
}

bool CeresPlanner::initializeWaypoints()
{
    const bool isEuclidean2D = (taskSpaceType_ == TaskSpaceType::Euclidean2D);
    const char* unreachableMessage = isEuclidean2D
        ? "Straight line passes through unreachable workspace!"
        : "Straight line passes through unreachable task space!";
    const TaskSpaceInterpolator interpolator;

    for (int i = 0; i < numWaypoints_; ++i)
    {
        double t = static_cast<double>(i) / (numWaypoints_ - 1);
        const TaskSpaceCoordinates taskTargetCoordinates =
            interpolator.interpolate(startTaskSpaceCoordinates_, goalTaskSpaceCoordinates_, t);
        std::vector<double> taskTarget = flattenTaskSpaceCoordinates(taskTargetCoordinates);

        JointManifoldState seedState = arm_->interpolateManifoldState(startState_, goalState_, t);

        JointManifoldState ikSolution;
        const JointManifoldState *initialState = &seedState;
        if (arm_->supportsInverseKinematics(taskSpaceType_))
        {
            if (!arm_->computeInverseKinematics(taskTarget, seedState, ikSolution))
            {
                std::cerr << unreachableMessage << std::endl;
                return false;
            }
            initialState = &ikSolution;
        }

        assignWaypointFromState(i, *initialState);
    }
    return true;
}

bool CeresPlanner::optimizePath(const std::vector<double> &direction, double solveTimeSeconds)
{
    const bool isEuclidean2D = (taskSpaceType_ == TaskSpaceType::Euclidean2D);
    if (!isEuclidean2D && jointCount_ < 3)
    {
        return false;
    }

    double p00 = 0.0;
    double p01 = 0.0;
    double p11 = 0.0;
    std::array<double, 16> projection{};
    if (isEuclidean2D)
    {
        p00 = 1.0 - direction[0] * direction[0];
        p01 = -direction[0] * direction[1];
        p11 = 1.0 - direction[1] * direction[1];
    }
    else
    {
        projection = buildProjectionMatrix4D(direction);
    }

    ceres::Problem::Options problemOptions;
    problemOptions.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(problemOptions);
    auto sphereManifold = std::make_unique<ceres::SphereManifold<2>>();

    const KinematicParameters kinParams = arm_->getKinematicParameters();
    const std::size_t requiredJointParams = isEuclidean2D ? 2U : 3U;
    if (kinParams.linkLengths.size() < requiredJointParams) return false;
    double l1 = kinParams.linkLengths[0];
    double l2 = kinParams.linkLengths[1];
    double l3 = isEuclidean2D ? 0.0 : kinParams.linkLengths[2];

    for (int i = 0; i < numWaypoints_; ++i)
    {
        std::vector<double*> parameterBlocks;
        parameterBlocks.reserve(jointCount_);
        for (std::size_t joint = 0; joint < jointCount_; ++joint)
        {
            double* block = jointPathBlock(joint, i);
            problem.AddParameterBlock(block, 2, sphereManifold.get());
            parameterBlocks.push_back(block);
        }

        ceres::CostFunction* lineCost = nullptr;
        if (isEuclidean2D)
        {
            lineCost = new ceres::AutoDiffCostFunction<StraightLineCost2D, 2, 2, 2>(
                new StraightLineCost2D(l1, l2, startTaskSpace_[0], startTaskSpace_[1], p00, p01, p11, options_.lineConstraintWeight));
        }
        else
        {
            lineCost = new ceres::AutoDiffCostFunction<StraightLineCost4D, 4, 2, 2, 2>(
                new StraightLineCost4D(l1, l2, l3,
                                       startTaskSpace_[0], startTaskSpace_[1], startTaskSpace_[2], startTaskSpace_[3],
                                       projection, options_.lineConstraintWeight));
        }
        problem.AddResidualBlock(lineCost, nullptr, parameterBlocks);
    }

    for (int i = 0; i < numWaypoints_ - 1; ++i)
    {
        for (std::size_t joint = 0; joint < jointCount_; ++joint)
        {
            auto* smoothCost = new ceres::AutoDiffCostFunction<SmoothnessCost, 2, 2, 2>(new SmoothnessCost(options_.smoothnessWeight));
            std::vector<double*> smoothBlocks = {
                jointPathBlock(joint, i),
                jointPathBlock(joint, i + 1)
            };
            problem.AddResidualBlock(smoothCost, nullptr, smoothBlocks);
        }
    }

    for (std::size_t joint = 0; joint < jointCount_; ++joint)
    {
        problem.SetParameterBlockConstant(jointPathBlock(joint, 0));
        problem.SetParameterBlockConstant(jointPathBlock(joint, numWaypoints_ - 1));
    }

    return runSolver(problem, solveTimeSeconds);
}

bool CeresPlanner::runSolver(ceres::Problem &problem, double solveTimeSeconds) const
{
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = options_.maxNumIterations;
    if (solveTimeSeconds > 0.0)
    {
        options.max_solver_time_in_seconds = solveTimeSeconds;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    return summary.IsSolutionUsable();
}

bool CeresPlanner::validatePath() const
{
    for (int i = 0; i < numWaypoints_; ++i)
    {
        JointManifoldState flatState = buildFlatState(i);
        if (!checker_->isManifoldStateValid(flatState))
        {
            std::cerr << "Collision detected on straight line at step " << i << std::endl;
            return false;
        }
    }
    return true;
}

ManifoldPath CeresPlanner::getPathManifoldStates() const
{
    ManifoldPath path;
    path.reserve(numWaypoints_);
    for (int i = 0; i < numWaypoints_; ++i)
    {
        JointManifoldState flatState = buildFlatState(i);
        path.push_back(std::move(flatState));
    }
    return path;
}

void CeresPlanner::assignWaypointFromState(int waypointIndex, const JointManifoldState &state)
{
    for (std::size_t joint = 0; joint < jointCount_; ++joint)
    {
        const std::size_t stateOffset = 2 * joint;
        jointPaths_[joint][waypointIndex][0] = state[stateOffset];
        jointPaths_[joint][waypointIndex][1] = state[stateOffset + 1];
    }
}

double* CeresPlanner::jointPathBlock(std::size_t jointIndex, int waypointIndex)
{
    return jointPaths_[jointIndex][waypointIndex].data();
}

JointManifoldState CeresPlanner::buildFlatState(int waypointIndex) const
{
    JointManifoldState state;
    state.reserve(2 * jointCount_);
    for (std::size_t joint = 0; joint < jointCount_; ++joint)
    {
        state.push_back(jointPaths_[joint][waypointIndex][0]);
        state.push_back(jointPaths_[joint][waypointIndex][1]);
    }
    return state;
}

double CeresPlanner::getPathLength() const
{
    double len = 0.0;
    auto path = getPathManifoldStates();
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        len += arm_->computeManifoldDistance(path[i], path[i-1]);
    }
    return len;
}

}  // namespace motion_planning_examples
