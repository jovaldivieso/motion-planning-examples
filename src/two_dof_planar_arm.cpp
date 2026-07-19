#include "two_dof_planar_arm.hpp"

#include <cmath>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ob = ompl::base;

namespace motion_planning_examples
{

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

void TwoDOFPlanarArm::computeEndEffectorTransform(const ob::State* state, fcl::Transform3d& transform) const
{
    const auto* compound = state->as<ob::CompoundStateSpace::StateType>();
    const double theta1 = compound->as<ob::SO2StateSpace::StateType>(0)->value;
    const double theta2 = compound->as<ob::SO2StateSpace::StateType>(1)->value;

    const double jointX = l1_ * std::cos(theta1);
    const double jointY = l1_ * std::sin(theta1);
    const double link2Yaw = theta1 + theta2;

    // Use full l2_ length to find the actual tip, not 0.5 * l2_
    transform = fcl::Transform3d::Identity();
    transform.linear() = Eigen::AngleAxisd(link2Yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    transform.translation() << jointX + l2_ * std::cos(link2Yaw), 
                               jointY + l2_ * std::sin(link2Yaw), 0.0;
}

std::vector<std::shared_ptr<fcl::CollisionGeometryd>> TwoDOFPlanarArm::getCollisionGeometries() const
{
    return {link1Geometry_, link2Geometry_};
}

bool TwoDOFPlanarArm::computeInverseKinematics(const std::vector<double>& targetWorkspace, bool hint, std::vector<double>& seedState) const
{
    if (targetWorkspace.size() < 2) return false;
    double px = targetWorkspace[0];
    double py = targetWorkspace[1];

    double p_sq = px * px + py * py;
    double k = (p_sq + l1_ * l1_ - l2_ * l2_) / (2.0 * l1_);
    double discriminant = p_sq - k * k;

    if (discriminant < 0.0) return false;

    double sign = hint ? 1.0 : -1.0;

    double v1x = (k * px + sign * std::sqrt(discriminant) * py) / p_sq;
    double v1y = (k * py - sign * std::sqrt(discriminant) * px) / p_sq;

    double v12x = (px - l1_ * v1x) / l2_;
    double v12y = (py - l1_ * v1y) / l2_;

    double v2x =  v1x * v12x + v1y * v12y;
    double v2y = -v1y * v12x + v1x * v12y;

    seedState = {v1x, v1y, v2x, v2y};
    return true;
}

std::vector<double> TwoDOFPlanarArm::getKinematicParameters() const
{
    return {l1_, l2_};
}

}  // namespace motion_planning_examples
