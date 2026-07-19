#include "ceres_planner.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <ceres/ceres.h>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ob = ompl::base;

namespace motion_planning_examples
{

// Functor to enforce orthogonal projection straight line constraint
struct StraightLineCost
{
    StraightLineCost(double l1, double l2, double sx, double sy, double p00, double p01, double p11, double weight)
        : l1_(l1), l2_(l2), sx_(sx), sy_(sy), p00_(p00), p01_(p01), p11_(p11), w_(weight) {}

    template <typename T>
    bool operator()(const T* const v1, const T* const v2, T* residual) const
    {
        // Compute forward kinematics completely algebraicaly: L1*v1 + L2*v12
        // where local v2 is projected to global v12 via the R1 rotation matrix
        T v12_x = v1[0] * v2[0] - v1[1] * v2[1];
        T v12_y = v1[1] * v2[0] + v1[0] * v2[1];
        
        T ee_x = T(l1_) * v1[0] + T(l2_) * v12_x;
        T ee_y = T(l1_) * v1[1] + T(l2_) * v12_y;
        
        T dx = ee_x - T(sx_);
        T dy = ee_y - T(sy_);

        // Apply Orthogonal Projection Matrix P_perp * delta_x
        residual[0] = T(w_) * (T(p00_) * dx + T(p01_) * dy);
        residual[1] = T(w_) * (T(p01_) * dx + T(p11_) * dy);
        return true;
    }

private:
    double l1_, l2_, sx_, sy_, p00_, p01_, p11_, w_;
};

// Functor to pull points evenly like an elastic band
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

CeresPlanner::CeresPlanner(std::shared_ptr<TwoDOFPlanarArm> arm, 
                           std::shared_ptr<FCLCollisionChecker> checker,
                           double link1Length, double link2Length, int numWaypoints)
    : arm_(std::move(arm)), checker_(std::move(checker)), 
      l1_(link1Length), l2_(link2Length), numWaypoints_(numWaypoints)
{
    if (numWaypoints_ < 2) throw std::invalid_argument("At least 2 waypoints required.");
    v1Path_.resize(2 * numWaypoints_, 0.0);
    v2Path_.resize(2 * numWaypoints_, 0.0);
}

void CeresPlanner::setStartGoalWorkspace(double startX, double startY, double goalX, double goalY, bool elbowUp)
{
    startX_ = startX;
    startY_ = startY;
    goalX_ = goalX;
    goalY_ = goalY;
    elbowUp_ = elbowUp;
}

bool CeresPlanner::computeAlgebraicIK(double px, double py, bool elbow_up, double& v1x, double& v1y, double& v2x, double& v2y) const
{
    double p_sq = px * px + py * py;
    double k = (p_sq + l1_ * l1_ - l2_ * l2_) / (2.0 * l1_);
    double discriminant = p_sq - k * k;

    if (discriminant < 0.0) return false; // Geometrically unreachable

    // By selecting the sign of the orthogonal tangent projection, we naturally 
    // select the elbow-up (+1.0) or elbow-down (-1.0) configuration branch natively on S1
    double sign = elbow_up ? 1.0 : -1.0;

    // 1. Intersect line and circle for link 1 
    v1x = (k * px + sign * std::sqrt(discriminant) * py) / p_sq;
    v1y = (k * py - sign * std::sqrt(discriminant) * px) / p_sq;

    // 2. Global link 2 vector
    double v12x = (px - l1_ * v1x) / l2_;
    double v12y = (py - l1_ * v1y) / l2_;

    // 3. Project to local joint space using R1^T
    v2x =  v1x * v12x + v1y * v12y;
    v2y = -v1y * v12x + v1x * v12y;

    return true;
}

bool CeresPlanner::solve()
{
    // 1. Build the math trick geometry
    double dirX = goalX_ - startX_;
    double dirY = goalY_ - startY_;
    double len = std::sqrt(dirX * dirX + dirY * dirY);
    if (len < 1e-6) return false;

    dirX /= len;
    dirY /= len;

    // Projection matrix P_perp = I - v*v^T
    double p00 = 1.0 - dirX * dirX;
    double p01 = -dirX * dirY;
    double p11 = 1.0 - dirY * dirY;

    // 2. Initial Seed Generation using pure algebraic IK mapping directly to S1
    for (int i = 0; i < numWaypoints_; ++i)
    {
        double t = static_cast<double>(i) / (numWaypoints_ - 1);
        double px = startX_ + t * (goalX_ - startX_);
        double py = startY_ + t * (goalY_ - startY_);

        if (!computeAlgebraicIK(px, py, elbowUp_,
                                v1Path_[2*i], v1Path_[2*i + 1], 
                                v2Path_[2*i], v2Path_[2*i + 1]))
        {
            std::cerr << "Straight line passes through unreachable workspace!" << std::endl;
            return false;
        }
    }

    // 3. Configure NLP directly on the Torus Manifold
    ceres::Problem problem;
    ceres::Manifold* sphereManifold = new ceres::SphereManifold<2>;

    for (int i = 0; i < numWaypoints_; ++i)
    {
        problem.AddParameterBlock(&v1Path_[2*i], 2, sphereManifold);
        problem.AddParameterBlock(&v2Path_[2*i], 2, sphereManifold);

        auto* lineCost = new ceres::AutoDiffCostFunction<StraightLineCost, 2, 2, 2>(
            new StraightLineCost(l1_, l2_, startX_, startY_, p00, p01, p11, 20.0));
        problem.AddResidualBlock(lineCost, nullptr, &v1Path_[2*i], &v2Path_[2*i]);
    }

    for (int i = 0; i < numWaypoints_ - 1; ++i)
    {
        auto* smoothCost1 = new ceres::AutoDiffCostFunction<SmoothnessCost, 2, 2, 2>(new SmoothnessCost(1.0));
        problem.AddResidualBlock(smoothCost1, nullptr, &v1Path_[2*i], &v1Path_[2*i + 2]);

        auto* smoothCost2 = new ceres::AutoDiffCostFunction<SmoothnessCost, 2, 2, 2>(new SmoothnessCost(1.0));
        problem.AddResidualBlock(smoothCost2, nullptr, &v2Path_[2*i], &v2Path_[2*i + 2]);
    }

    // Pin the start and goal strictly to their IK solutions
    problem.SetParameterBlockConstant(&v1Path_[0]);
    problem.SetParameterBlockConstant(&v2Path_[0]);
    problem.SetParameterBlockConstant(&v1Path_[2*(numWaypoints_-1)]);
    problem.SetParameterBlockConstant(&v2Path_[2*(numWaypoints_-1)]);

    // 4. Solve 
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 100;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    if (!summary.IsSolutionUsable()) return false;

    // 5. Collision Avoidance Sweep along the computed line
    ob::State* state = arm_->getStateSpace()->allocState();
    auto* compound = state->as<ob::CompoundStateSpace::StateType>();

    bool valid = true;
    for (int i = 0; i < numWaypoints_; ++i)
    {
        compound->as<ob::SO2StateSpace::StateType>(0)->value = std::atan2(v1Path_[2*i + 1], v1Path_[2*i]);
        compound->as<ob::SO2StateSpace::StateType>(1)->value = std::atan2(v2Path_[2*i + 1], v2Path_[2*i]);
        
        if (!checker_->isStateValid(state))
        {
            std::cerr << "Collision detected on straight line at step " << i << std::endl;
            valid = false;
            break;
        }
    }
    
    arm_->getStateSpace()->freeState(state);
    return valid;
}

std::vector<std::pair<double, double>> CeresPlanner::getPathAngles() const
{
    std::vector<std::pair<double, double>> path;
    path.reserve(numWaypoints_);
    for (int i = 0; i < numWaypoints_; ++i)
    {
        // Extract the explicit SO(2) manifold vectors back to angles purely for execution
        double t1 = std::atan2(v1Path_[2*i + 1], v1Path_[2*i]);
        double t2 = std::atan2(v2Path_[2*i + 1], v2Path_[2*i]);
        path.emplace_back(t1, t2);
    }
    return path;
}

}  // namespace motion_planning_examples
