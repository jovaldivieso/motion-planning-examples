#pragma once

#include <memory>
#include <fcl/fcl.h>
#include <ompl/base/StateSpace.h>

namespace motion_planning_examples
{

class TwoDOFPlanarArm
{
public:
    TwoDOFPlanarArm(double link1Length, double link2Length, double linkThickness, double objectHeight);

    // Returns the SO(2) x SO(2) compound state space defining this arm
    [[nodiscard]] std::shared_ptr<ompl::base::StateSpace> getStateSpace() const;

    // Forward Kinematics: Computes the 3D transforms for both links given a state
    void computeForwardKinematics(const ompl::base::State* state, 
                                  fcl::Transform3d& tf1, 
                                  fcl::Transform3d& tf2) const;

    // Geometries required by the collision checker
    [[nodiscard]] std::shared_ptr<fcl::CollisionGeometryd> getLink1Geometry() const { return link1Geometry_; }
    [[nodiscard]] std::shared_ptr<fcl::CollisionGeometryd> getLink2Geometry() const { return link2Geometry_; }

private:
    double l1_;
    double l2_;
    
    std::shared_ptr<ompl::base::StateSpace> space_;
    std::shared_ptr<fcl::CollisionGeometryd> link1Geometry_;
    std::shared_ptr<fcl::CollisionGeometryd> link2Geometry_;
};

}  // namespace motion_planning_examples
