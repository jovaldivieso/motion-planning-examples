#pragma once

#include "robot_mechanism.hpp"

namespace motion_planning_examples
{

class TwoDOFPlanarArm : public RobotMechanism
{
public:
    TwoDOFPlanarArm(double link1Length, double link2Length, double linkThickness, double objectHeight);

    [[nodiscard]] std::shared_ptr<ompl::base::StateSpace> getStateSpace() const override;

    void computeForwardKinematics(const ompl::base::State* state, 
                                  std::vector<fcl::Transform3d>& transforms) const override;

    void computeForwardKinematicsFromManifoldState(const JointManifoldState &state,
                                                   std::vector<fcl::Transform3d> &transforms) const override;

    void computeEndEffectorTransform(const ompl::base::State* state, 
                                     fcl::Transform3d& transform) const override;

    void computeEndEffectorFromManifoldState(const JointManifoldState &state,
                                             fcl::Transform3d &transform) const override;

    [[nodiscard]] std::vector<std::shared_ptr<fcl::CollisionGeometryd>> getCollisionGeometries() const override;

    [[nodiscard]] bool computeInverseKinematics(const std::vector<double>& targetWorkspace,
                                                const JointManifoldState &seedState,
                                                JointManifoldState &solutionState) const override;

    [[nodiscard]] std::vector<double> getKinematicParameters() const override;

private:
    double l1_;
    double l2_;
    
    std::shared_ptr<ompl::base::StateSpace> space_;
    std::shared_ptr<fcl::CollisionGeometryd> link1Geometry_;
    std::shared_ptr<fcl::CollisionGeometryd> link2Geometry_;
};

}  // namespace motion_planning_examples