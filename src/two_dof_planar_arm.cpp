#include "two_dof_planar_arm.hpp"

#include <cmath>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ob = ompl::base;

namespace motion_planning_examples
{

TwoDOFPlanarArm::TwoDOFPlanarArm(double link1Length, double link2Length, double linkThickness, double objectHeight)
    : l1_(link1Length), l2_(link2Length)
{
    // 1. Construct the State Space
    auto compound = std::make_shared<ob::CompoundStateSpace>();
    compound->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
    compound->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
    compound->lock();
    space_ = compound;

    // 2. Construct link geometries once
    link1Geometry_ = std::make_shared<fcl::Boxd>(l1_, linkThickness, objectHeight);
    link2Geometry_ = std::make_shared<fcl::Boxd>(l2_, linkThickness, objectHeight);
}

std::shared_ptr<ob::StateSpace> TwoDOFPlanarArm::getStateSpace() const
{
    return space_;
}

void TwoDOFPlanarArm::computeForwardKinematics(const ob::State* state, fcl::Transform3d& tf1, fcl::Transform3d& tf2) const
{
    const auto* compound = state->as<ob::CompoundStateSpace::StateType>();
    const double theta1 = compound->as<ob::SO2StateSpace::StateType>(0)->value;
    const double theta2 = compound->as<ob::SO2StateSpace::StateType>(1)->value;

    // Transform for Link 1
    tf1 = fcl::Transform3d::Identity();
    tf1.linear() = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    tf1.translation() << 0.5 * l1_ * std::cos(theta1), 0.5 * l1_ * std::sin(theta1), 0.0;

    // Transform for Link 2
    const double jointX = l1_ * std::cos(theta1);
    const double jointY = l1_ * std::sin(theta1);
    const double link2Yaw = theta1 + theta2;
    
    tf2 = fcl::Transform3d::Identity();
    tf2.linear() = Eigen::AngleAxisd(link2Yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    tf2.translation() << jointX + 0.5 * l2_ * std::cos(link2Yaw), 
                         jointY + 0.5 * l2_ * std::sin(link2Yaw), 0.0;
}

}  // namespace motion_planning_examples
