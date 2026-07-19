#include "two_dof_planar_arm.hpp"

#include <algorithm>
#include <cmath>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ob = ompl::base;

namespace motion_planning_examples
{

namespace
{

JointS1 unitFromAngle(double theta)
{
    return {std::cos(theta), std::sin(theta)};
}
double manifoldDistanceOnS1(const JointS1 &a, const JointS1 &b)
{
    const double dot = std::clamp(a[0] * b[0] + a[1] * b[1], -1.0, 1.0);
    return std::acos(dot);
}

}  // namespace

TwoDOFPlanarArm::TwoDOFPlanarArm(double link1Length, double link2Length, double linkThickness, double objectHeight)
    : l1_(link1Length), l2_(link2Length)
{
    auto compound = std::make_shared<ob::CompoundStateSpace>();
    compound->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
    compound->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
    compound->lock();
    space_ = compound;

    link1Geometry_ = std::make_shared<fcl::Boxd>(l1_, linkThickness, objectHeight);
    link2Geometry_ = std::make_shared<fcl::Boxd>(l2_, linkThickness, objectHeight);
}

std::shared_ptr<ob::StateSpace> TwoDOFPlanarArm::getStateSpace() const
{
    return space_;
}

void TwoDOFPlanarArm::computeForwardKinematics(const ob::State* state, std::vector<fcl::Transform3d>& transforms) const
{
    const auto* compound = state->as<ob::CompoundStateSpace::StateType>();
    const double theta1 = compound->as<ob::SO2StateSpace::StateType>(0)->value;
    const double theta2 = compound->as<ob::SO2StateSpace::StateType>(1)->value;

    fcl::Transform3d tf1 = fcl::Transform3d::Identity();
    tf1.linear() = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    tf1.translation() << 0.5 * l1_ * std::cos(theta1), 0.5 * l1_ * std::sin(theta1), 0.0;

    const double jointX = l1_ * std::cos(theta1);
    const double jointY = l1_ * std::sin(theta1);
    const double link2Yaw = theta1 + theta2;
    
    fcl::Transform3d tf2 = fcl::Transform3d::Identity();
    tf2.linear() = Eigen::AngleAxisd(link2Yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    tf2.translation() << jointX + 0.5 * l2_ * std::cos(link2Yaw), 
                         jointY + 0.5 * l2_ * std::sin(link2Yaw), 0.0;

    transforms.clear();
    transforms.push_back(tf1);
    transforms.push_back(tf2);
}

void TwoDOFPlanarArm::computeForwardKinematicsFromManifoldState(const JointManifoldState &state,
                                                                std::vector<fcl::Transform3d> &transforms) const
{
    if (state.size() < 2)
    {
        transforms.clear();
        return;
    }

    const JointS1 &v1 = state[0];
    const JointS1 &v2 = state[1];

    const double c1 = v1[0];
    const double s1 = v1[1];
    const double c12 = c1 * v2[0] - s1 * v2[1];
    const double s12 = s1 * v2[0] + c1 * v2[1];

    fcl::Transform3d tf1 = fcl::Transform3d::Identity();
    tf1.linear() = Eigen::AngleAxisd(std::atan2(s1, c1), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    tf1.translation() << 0.5 * l1_ * c1, 0.5 * l1_ * s1, 0.0;

    const double jointX = l1_ * c1;
    const double jointY = l1_ * s1;

    fcl::Transform3d tf2 = fcl::Transform3d::Identity();
    tf2.linear() = Eigen::AngleAxisd(std::atan2(s12, c12), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    tf2.translation() << jointX + 0.5 * l2_ * c12,
                         jointY + 0.5 * l2_ * s12, 0.0;

    transforms.clear();
    transforms.push_back(tf1);
    transforms.push_back(tf2);
}

void TwoDOFPlanarArm::computeEndEffectorTransform(const ob::State* state, fcl::Transform3d& transform) const
{
    const auto* compound = state->as<ob::CompoundStateSpace::StateType>();
    const double theta1 = compound->as<ob::SO2StateSpace::StateType>(0)->value;
    const double theta2 = compound->as<ob::SO2StateSpace::StateType>(1)->value;

    const double jointX = l1_ * std::cos(theta1);
    const double jointY = l1_ * std::sin(theta1);
    const double link2Yaw = theta1 + theta2;

    transform = fcl::Transform3d::Identity();
    transform.linear() = Eigen::AngleAxisd(link2Yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    transform.translation() << jointX + l2_ * std::cos(link2Yaw), 
                               jointY + l2_ * std::sin(link2Yaw), 0.0;
}

void TwoDOFPlanarArm::computeEndEffectorFromManifoldState(const JointManifoldState &state,
                                                           fcl::Transform3d &transform) const
{
    if (state.size() < 2)
    {
        transform = fcl::Transform3d::Identity();
        return;
    }

    const JointS1 &v1 = state[0];
    const JointS1 &v2 = state[1];

    const double c1 = v1[0];
    const double s1 = v1[1];
    const double c12 = c1 * v2[0] - s1 * v2[1];
    const double s12 = s1 * v2[0] + c1 * v2[1];

    transform = fcl::Transform3d::Identity();
    transform.linear() = Eigen::AngleAxisd(std::atan2(s12, c12), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    transform.translation() << l1_ * c1 + l2_ * c12,
                               l1_ * s1 + l2_ * s12, 0.0;
}

std::vector<std::shared_ptr<fcl::CollisionGeometryd>> TwoDOFPlanarArm::getCollisionGeometries() const
{
    return {link1Geometry_, link2Geometry_};
}

bool TwoDOFPlanarArm::computeInverseKinematics(const std::vector<double>& targetWorkspace,
                                               const JointManifoldState &seedState,
                                               JointManifoldState &solutionState) const
{
    if (targetWorkspace.size() < 2 || seedState.size() < 2) return false;
    double px = targetWorkspace[0];
    double py = targetWorkspace[1];

    double p_sq = px * px + py * py;
    double val = (p_sq - l1_ * l1_ - l2_ * l2_) / (2.0 * l1_ * l2_);
    
    if (val < -1.0 || val > 1.0) return false; // Unreachable

    // Solution A (Elbow down)
    double theta2_a = std::acos(val);
    double theta1_a = std::atan2(py, px) - std::atan2(l2_ * std::sin(theta2_a), l1_ + l2_ * std::cos(theta2_a));

    // Solution B (Elbow up)
    double theta2_b = -std::acos(val);
    double theta1_b = std::atan2(py, px) - std::atan2(l2_ * std::sin(theta2_b), l1_ + l2_ * std::cos(theta2_b));

    const JointManifoldState candidateA = {unitFromAngle(theta1_a), unitFromAngle(theta2_a)};
    const JointManifoldState candidateB = {unitFromAngle(theta1_b), unitFromAngle(theta2_b)};

    // Pick solution closest to the provided seed
    double dist_a = std::pow(manifoldDistanceOnS1(candidateA[0], seedState[0]), 2)
                  + std::pow(manifoldDistanceOnS1(candidateA[1], seedState[1]), 2);
    double dist_b = std::pow(manifoldDistanceOnS1(candidateB[0], seedState[0]), 2)
                  + std::pow(manifoldDistanceOnS1(candidateB[1], seedState[1]), 2);

    if (dist_a <= dist_b)
    {
        solutionState = candidateA;
    } else {
        solutionState = candidateB;
    }
    
    return true;
}

std::vector<double> TwoDOFPlanarArm::getKinematicParameters() const
{
    return {l1_, l2_};
}

}  // namespace motion_planning_examples