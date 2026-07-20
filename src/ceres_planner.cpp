#include "ceres_planner.hpp"

#include <cmath>
#include <iostream>
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
                           int numWaypoints)
    : arm_(std::move(arm)), checker_(std::move(checker)), 
            numWaypoints_(numWaypoints),
            jointCount_(arm_->getJointCount())
{
        if (numWaypoints_ < 2) throw std::invalid_argument("At least 2 waypoints required.");
        if (jointCount_ < 2 || jointCount_ > 3) throw std::invalid_argument("CeresPlanner currently supports 2 or 3 planar joints.");
    v1Path_.resize(2 * numWaypoints_, 0.0);
    v2Path_.resize(2 * numWaypoints_, 0.0);
        if (jointCount_ >= 3) v3Path_.resize(2 * numWaypoints_, 0.0);
}

void CeresPlanner::setStartGoal(const JointManifoldState &start, const JointManifoldState &goal)
{
    startState_ = start;
    goalState_ = goal;

    startTaskSpace_ = arm_->computeTaskSpaceCoordinates(startState_);
    goalTaskSpace_ = arm_->computeTaskSpaceCoordinates(goalState_);

    if (startTaskSpace_.size() != goalTaskSpace_.size())
    {
        throw std::invalid_argument("Start and goal task-space dimensions must match.");
    }

    taskSpaceType_ = inferTaskSpaceType(startTaskSpace_.size());
    taskSpaceDimension_ = startTaskSpace_.size();
    if (taskSpaceType_ != TaskSpaceType::Euclidean2D && taskSpaceType_ != TaskSpaceType::SE2)
    {
        throw std::invalid_argument("CeresPlanner currently supports 2D Euclidean or SE(2) task spaces.");
    }
}

bool CeresPlanner::solve(double /*solveTimeSeconds*/)
{
    std::vector<double> dir(taskSpaceDimension_, 0.0);
    for (std::size_t i = 0; i < taskSpaceDimension_; ++i)
    {
        dir[i] = goalTaskSpace_[i] - startTaskSpace_[i];
    }

    double len = 0.0;
    for (double value : dir) len += value * value;
    len = std::sqrt(len);
    if (len < 1e-6) return false;

    for (double &value : dir) value /= len;

    if (taskSpaceType_ == TaskSpaceType::Euclidean2D)
    {
        double p00 = 1.0 - dir[0] * dir[0];
        double p01 = -dir[0] * dir[1];
        double p11 = 1.0 - dir[1] * dir[1];

        for (int i = 0; i < numWaypoints_; ++i)
        {
            double t = static_cast<double>(i) / (numWaypoints_ - 1);
            double px = startTaskSpace_[0] + t * (goalTaskSpace_[0] - startTaskSpace_[0]);
            double py = startTaskSpace_[1] + t * (goalTaskSpace_[1] - startTaskSpace_[1]);

            JointManifoldState seedState = arm_->interpolateManifoldState(startState_, goalState_, t);
            JointManifoldState ikSolution;
            const JointManifoldState *initialState = &seedState;
            if (arm_->supportsInverseKinematics(taskSpaceType_))
            {
                if (!arm_->computeInverseKinematics({px, py}, seedState, ikSolution))
                {
                    std::cerr << "Straight line passes through unreachable workspace!" << std::endl;
                    return false;
                }
                initialState = &ikSolution;
            }

            v1Path_[2 * i] = (*initialState)[0];
            v1Path_[2 * i + 1] = (*initialState)[1];
            v2Path_[2 * i] = (*initialState)[2];
            v2Path_[2 * i + 1] = (*initialState)[3];
        }

        ceres::Problem problem;
        ceres::Manifold* sphereManifold = new ceres::SphereManifold<2>;

        std::vector<double> kinParams = arm_->getKinematicParameters();
        if (kinParams.size() < 2) return false;
        double l1 = kinParams[0];
        double l2 = kinParams[1];

        for (int i = 0; i < numWaypoints_; ++i)
        {
            problem.AddParameterBlock(&v1Path_[2*i], 2, sphereManifold);
            problem.AddParameterBlock(&v2Path_[2*i], 2, sphereManifold);

            auto* lineCost = new ceres::AutoDiffCostFunction<StraightLineCost2D, 2, 2, 2>(
                new StraightLineCost2D(l1, l2, startTaskSpace_[0], startTaskSpace_[1], p00, p01, p11, 20.0));
            problem.AddResidualBlock(lineCost, nullptr, &v1Path_[2*i], &v2Path_[2*i]);
        }

        for (int i = 0; i < numWaypoints_ - 1; ++i)
        {
            auto* smoothCost1 = new ceres::AutoDiffCostFunction<SmoothnessCost, 2, 2, 2>(new SmoothnessCost(1.0));
            problem.AddResidualBlock(smoothCost1, nullptr, &v1Path_[2*i], &v1Path_[2*i + 2]);

            auto* smoothCost2 = new ceres::AutoDiffCostFunction<SmoothnessCost, 2, 2, 2>(new SmoothnessCost(1.0));
            problem.AddResidualBlock(smoothCost2, nullptr, &v2Path_[2*i], &v2Path_[2*i + 2]);
        }

        problem.SetParameterBlockConstant(&v1Path_[0]);
        problem.SetParameterBlockConstant(&v2Path_[0]);
        problem.SetParameterBlockConstant(&v1Path_[2*(numWaypoints_-1)]);
        problem.SetParameterBlockConstant(&v2Path_[2*(numWaypoints_-1)]);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 100;
        
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        
        if (!summary.IsSolutionUsable()) return false;

        bool valid = true;
        for (int i = 0; i < numWaypoints_; ++i)
        {
            JointManifoldState flatState = {v1Path_[2*i], v1Path_[2*i+1], v2Path_[2*i], v2Path_[2*i+1]};
            if (!checker_->isManifoldStateValid(flatState))
            {
                std::cerr << "Collision detected on straight line at step " << i << std::endl;
                valid = false;
                break;
            }
        }
        return valid;
    }

    const std::array<double, 16> projection = buildProjectionMatrix4D(dir);

    for (int i = 0; i < numWaypoints_; ++i)
    {
        double t = static_cast<double>(i) / (numWaypoints_ - 1);
        std::vector<double> taskTarget = interpolateTaskSpaceCoordinates(taskSpaceType_, startTaskSpace_, goalTaskSpace_, t);

        JointManifoldState seedState = arm_->interpolateManifoldState(startState_, goalState_, t);

        JointManifoldState ikSolution;
        const JointManifoldState *initialState = &seedState;
        if (arm_->supportsInverseKinematics(taskSpaceType_))
        {
            if (!arm_->computeInverseKinematics(taskTarget, seedState, ikSolution))
            {
                std::cerr << "Straight line passes through unreachable task space!" << std::endl;
                return false;
            }
            initialState = &ikSolution;
        }

        v1Path_[2 * i] = (*initialState)[0];
        v1Path_[2 * i + 1] = (*initialState)[1];
        v2Path_[2 * i] = (*initialState)[2];
        v2Path_[2 * i + 1] = (*initialState)[3];
        if (jointCount_ >= 3)
        {
            v3Path_[2 * i] = (*initialState)[4];
            v3Path_[2 * i + 1] = (*initialState)[5];
        }
    }

    ceres::Problem problem;
    ceres::Manifold* sphereManifold = new ceres::SphereManifold<2>;

    std::vector<double> kinParams = arm_->getKinematicParameters();
    if (kinParams.size() < 3) return false;
    double l1 = kinParams[0];
    double l2 = kinParams[1];
    double l3 = kinParams[2];

    for (int i = 0; i < numWaypoints_; ++i)
    {
        problem.AddParameterBlock(&v1Path_[2*i], 2, sphereManifold);
        problem.AddParameterBlock(&v2Path_[2*i], 2, sphereManifold);
        problem.AddParameterBlock(&v3Path_[2*i], 2, sphereManifold);

        auto* lineCost = new ceres::AutoDiffCostFunction<StraightLineCost4D, 4, 2, 2, 2>(
            new StraightLineCost4D(l1, l2, l3,
                                   startTaskSpace_[0], startTaskSpace_[1], startTaskSpace_[2], startTaskSpace_[3],
                                   projection, 20.0));
        problem.AddResidualBlock(lineCost, nullptr, &v1Path_[2*i], &v2Path_[2*i], &v3Path_[2*i]);
    }

    for (int i = 0; i < numWaypoints_ - 1; ++i)
    {
        auto* smoothCost1 = new ceres::AutoDiffCostFunction<SmoothnessCost, 2, 2, 2>(new SmoothnessCost(1.0));
        problem.AddResidualBlock(smoothCost1, nullptr, &v1Path_[2*i], &v1Path_[2*i + 2]);

        auto* smoothCost2 = new ceres::AutoDiffCostFunction<SmoothnessCost, 2, 2, 2>(new SmoothnessCost(1.0));
        problem.AddResidualBlock(smoothCost2, nullptr, &v2Path_[2*i], &v2Path_[2*i + 2]);

        auto* smoothCost3 = new ceres::AutoDiffCostFunction<SmoothnessCost, 2, 2, 2>(new SmoothnessCost(1.0));
        problem.AddResidualBlock(smoothCost3, nullptr, &v3Path_[2*i], &v3Path_[2*i + 2]);
    }

    problem.SetParameterBlockConstant(&v1Path_[0]);
    problem.SetParameterBlockConstant(&v2Path_[0]);
    problem.SetParameterBlockConstant(&v3Path_[0]);
    problem.SetParameterBlockConstant(&v1Path_[2*(numWaypoints_-1)]);
    problem.SetParameterBlockConstant(&v2Path_[2*(numWaypoints_-1)]);
    problem.SetParameterBlockConstant(&v3Path_[2*(numWaypoints_-1)]);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 100;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    if (!summary.IsSolutionUsable()) return false;

    bool valid = true;
    for (int i = 0; i < numWaypoints_; ++i)
    {
        JointManifoldState flatState = {v1Path_[2*i], v1Path_[2*i+1], v2Path_[2*i], v2Path_[2*i+1]};
        if (jointCount_ >= 3)
        {
            flatState.push_back(v3Path_[2*i]);
            flatState.push_back(v3Path_[2*i+1]);
        }
        if (!checker_->isManifoldStateValid(flatState))
        {
            std::cerr << "Collision detected on straight line at step " << i << std::endl;
            valid = false;
            break;
        }
    }
    return valid;
}

ManifoldPath CeresPlanner::getPathManifoldStates() const
{
    ManifoldPath path;
    path.reserve(numWaypoints_);
    for (int i = 0; i < numWaypoints_; ++i)
    {
        JointManifoldState flatState = {v1Path_[2*i], v1Path_[2*i+1], v2Path_[2*i], v2Path_[2*i+1]};
        if (jointCount_ >= 3)
        {
            flatState.push_back(v3Path_[2*i]);
            flatState.push_back(v3Path_[2*i+1]);
        }
        path.push_back(std::move(flatState));
    }
    return path;
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
