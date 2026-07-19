#include "ceres_planner.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <ceres/ceres.h>

namespace motion_planning_examples
{

struct StraightLineCost
{
    StraightLineCost(double l1, double l2, double sx, double sy, double p00, double p01, double p11, double weight)
        : l1_(l1), l2_(l2), sx_(sx), sy_(sy), p00_(p00), p01_(p01), p11_(p11), w_(weight) {}

    template <typename T>
    bool operator()(const T* const v1, const T* const v2, T* residual) const
    {
        T v12_x = v1[0] * v2[0] - v1[1] * v2[1];
        T v12_y = v1[1] * v2[0] + v1[0] * v2[1];
        
        T ee_x = T(l1_) * v1[0] + T(l2_) * v12_x;
        T ee_y = T(l1_) * v1[1] + T(l2_) * v12_y;
        
        T dx = ee_x - T(sx_);
        T dy = ee_y - T(sy_);

        residual[0] = T(w_) * (T(p00_) * dx + T(p01_) * dy);
        residual[1] = T(w_) * (T(p01_) * dx + T(p11_) * dy);
        return true;
    }

private:
    double l1_, l2_, sx_, sy_, p00_, p01_, p11_, w_;
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
      numWaypoints_(numWaypoints)
{
    if (numWaypoints_ < 2) throw std::invalid_argument("At least 2 waypoints required.");
    v1Path_.resize(2 * numWaypoints_, 0.0);
    v2Path_.resize(2 * numWaypoints_, 0.0);
}

void CeresPlanner::setStartGoal(const JointManifoldState &start, const JointManifoldState &goal)
{
    startState_ = start;
    goalState_ = goal;

    fcl::Transform3d ee_tf;
    arm_->computeEndEffectorFromManifoldState(startState_, ee_tf);
    startX_ = ee_tf.translation().x();
    startY_ = ee_tf.translation().y();

    arm_->computeEndEffectorFromManifoldState(goalState_, ee_tf);
    goalX_ = ee_tf.translation().x();
    goalY_ = ee_tf.translation().y();
}

bool CeresPlanner::solve(double /*solveTimeSeconds*/)
{
    double dirX = goalX_ - startX_;
    double dirY = goalY_ - startY_;
    double len = std::sqrt(dirX * dirX + dirY * dirY);
    if (len < 1e-6) return false;

    dirX /= len;
    dirY /= len;

    double p00 = 1.0 - dirX * dirX;
    double p01 = -dirX * dirY;
    double p11 = 1.0 - dirY * dirY;

    for (int i = 0; i < numWaypoints_; ++i)
    {
        double t = static_cast<double>(i) / (numWaypoints_ - 1);
        double px = startX_ + t * (goalX_ - startX_);
        double py = startY_ + t * (goalY_ - startY_);

        JointManifoldState seedState = arm_->interpolateManifoldState(startState_, goalState_, t);

        JointManifoldState ikSolution;
        if (!arm_->computeInverseKinematics({px, py}, seedState, ikSolution))
        {
            std::cerr << "Straight line passes through unreachable workspace!" << std::endl;
            return false;
        }

        v1Path_[2 * i] = ikSolution[0];
        v1Path_[2 * i + 1] = ikSolution[1];
        v2Path_[2 * i] = ikSolution[2];
        v2Path_[2 * i + 1] = ikSolution[3];
    }

    ceres::Problem problem;
    ceres::Manifold* sphereManifold = new ceres::SphereManifold<2>;

    std::vector<double> kinParams = arm_->getKinematicParameters();
    double l1 = kinParams[0];
    double l2 = kinParams[1];

    for (int i = 0; i < numWaypoints_; ++i)
    {
        problem.AddParameterBlock(&v1Path_[2*i], 2, sphereManifold);
        problem.AddParameterBlock(&v2Path_[2*i], 2, sphereManifold);

        auto* lineCost = new ceres::AutoDiffCostFunction<StraightLineCost, 2, 2, 2>(
            new StraightLineCost(l1, l2, startX_, startY_, p00, p01, p11, 20.0));
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

ManifoldPath CeresPlanner::getPathManifoldStates() const
{
    ManifoldPath path;
    path.reserve(numWaypoints_);
    for (int i = 0; i < numWaypoints_; ++i)
    {
        path.push_back({v1Path_[2*i], v1Path_[2*i+1], v2Path_[2*i], v2Path_[2*i+1]});
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
