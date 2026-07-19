#include "ceres_planner.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <ceres/ceres.h>

namespace motion_planning_examples
{

namespace
{
JointS1 slerpS1(const JointS1 &a, const JointS1 &b, double t)
{
    const double dot = std::clamp(a[0] * b[0] + a[1] * b[1], -1.0, 1.0);
    const double omega = std::acos(dot);

    if (omega < 1e-10)
    {
        return a;
    }

    const double sinOmega = std::sin(omega);
    const double wa = std::sin((1.0 - t) * omega) / sinOmega;
    const double wb = std::sin(t * omega) / sinOmega;

    JointS1 out{wa * a[0] + wb * b[0], wa * a[1] + wb * b[1]};
    const double n = std::sqrt(out[0] * out[0] + out[1] * out[1]);
    if (n < 1e-12)
    {
        return a;
    }
    out[0] /= n;
    out[1] /= n;
    return out;
}

}  // namespace

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
    if (start.size() < 2 || goal.size() < 2)
    {
        throw std::invalid_argument("CeresPlanner expects at least 2 manifold joints for start and goal");
    }

    startState_ = start;
    goalState_ = goal;

    // Evaluate start/goal task-space via manifold-native FK.
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

    // Use manifold interpolation to guide IK branch continuity.
    for (int i = 0; i < numWaypoints_; ++i)
    {
        double t = static_cast<double>(i) / (numWaypoints_ - 1);
        double px = startX_ + t * (goalX_ - startX_);
        double py = startY_ + t * (goalY_ - startY_);

        JointManifoldState seedState = {
            slerpS1(startState_[0], goalState_[0], t),
            slerpS1(startState_[1], goalState_[1], t)
        };

        JointManifoldState ikSolution;
        if (!arm_->computeInverseKinematics({px, py}, seedState, ikSolution))
        {
            std::cerr << "Straight line passes through unreachable workspace!" << std::endl;
            return false;
        }

        // Ceres optimization acts directly on manifold unit vectors.
        v1Path_[2 * i] = ikSolution[0][0];
        v1Path_[2 * i + 1] = ikSolution[0][1];
        v2Path_[2 * i] = ikSolution[1][0];
        v2Path_[2 * i + 1] = ikSolution[1][1];
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
        JointManifoldState state = {
            JointS1{v1Path_[2 * i], v1Path_[2 * i + 1]},
            JointS1{v2Path_[2 * i], v2Path_[2 * i + 1]}
        };

        if (!checker_->isManifoldStateValid(state))
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
        path.push_back({
            JointS1{v1Path_[2 * i], v1Path_[2 * i + 1]},
            JointS1{v2Path_[2 * i], v2Path_[2 * i + 1]}
        });
    }
    return path;
}

double CeresPlanner::getPathLength() const
{
    double len = 0.0;
    auto path = getPathManifoldStates();
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        const double dot1 = std::clamp(path[i][0][0] * path[i - 1][0][0] + path[i][0][1] * path[i - 1][0][1], -1.0, 1.0);
        const double dot2 = std::clamp(path[i][1][0] * path[i - 1][1][0] + path[i][1][1] * path[i - 1][1][1], -1.0, 1.0);
        const double d1 = std::acos(dot1);
        const double d2 = std::acos(dot2);
        len += std::sqrt(d1 * d1 + d2 * d2);
    }
    return len;
}

}  // namespace motion_planning_examples