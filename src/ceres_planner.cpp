#include "ceres_planner.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <ceres/ceres.h>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ob = ompl::base;

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

void CeresPlanner::setStartGoal(double startTheta1, double startTheta2, double goalTheta1, double goalTheta2)
{
    auto space = arm_->getStateSpace();
    
    // Evaluate Start Task Space via generic EE FK
    ob::State* sState = space->allocState();
    auto* sComp = sState->as<ob::CompoundStateSpace::StateType>();
    sComp->as<ob::SO2StateSpace::StateType>(0)->value = startTheta1;
    sComp->as<ob::SO2StateSpace::StateType>(1)->value = startTheta2;
    
    fcl::Transform3d ee_tf;
    arm_->computeEndEffectorTransform(sState, ee_tf);
    startX_ = ee_tf.translation().x();
    startY_ = ee_tf.translation().y();
    
    // Evaluate Goal Task Space
    ob::State* gState = space->allocState();
    auto* gComp = gState->as<ob::CompoundStateSpace::StateType>();
    gComp->as<ob::SO2StateSpace::StateType>(0)->value = goalTheta1;
    gComp->as<ob::SO2StateSpace::StateType>(1)->value = goalTheta2;
    
    arm_->computeEndEffectorTransform(gState, ee_tf);
    goalX_ = ee_tf.translation().x();
    goalY_ = ee_tf.translation().y();

    space->freeState(sState);
    space->freeState(gState);

    elbowUp_ = (startTheta2 >= 0.0);
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

    // Delegate Seed IK directly to the specific mechanism implementation
    for (int i = 0; i < numWaypoints_; ++i)
    {
        double t = static_cast<double>(i) / (numWaypoints_ - 1);
        double px = startX_ + t * (goalX_ - startX_);
        double py = startY_ + t * (goalY_ - startY_);

        std::vector<double> seed;
        if (!arm_->computeInverseKinematics({px, py}, elbowUp_, seed))
        {
            std::cerr << "Straight line passes through unreachable workspace!" << std::endl;
            return false;
        }
        v1Path_[2*i] = seed[0];
        v1Path_[2*i + 1] = seed[1];
        v2Path_[2*i] = seed[2];
        v2Path_[2*i + 1] = seed[3];
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
        double t1 = std::atan2(v1Path_[2*i + 1], v1Path_[2*i]);
        double t2 = std::atan2(v2Path_[2*i + 1], v2Path_[2*i]);
        path.emplace_back(t1, t2);
    }
    return path;
}

double CeresPlanner::getPathLength() const
{
    double len = 0.0;
    auto path = getPathAngles();
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        double d1 = path[i].first - path[i-1].first;
        double d2 = path[i].second - path[i-1].second;
        len += std::sqrt(d1 * d1 + d2 * d2);
    }
    return len;
}

}  // namespace motion_planning_examples
