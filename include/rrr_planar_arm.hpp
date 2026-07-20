#pragma once

#include "robot_mechanism.hpp"

namespace motion_planning_examples
{

class RRRPlanarArm : public RobotMechanism
{
public:
  RRRPlanarArm(double link1Length, double link2Length, double link3Length, double linkThickness,
               double objectHeight);

  [[nodiscard]] std::size_t getJointCount() const override;

  [[nodiscard]] TaskSpaceCoordinates
  computeTaskSpaceCoordinates(const JointManifoldState& state) const override;

  void computeForwardKinematics(const JointManifoldState& state,
                                std::vector<fcl::Transform3d>& transforms) const override;

  void computeEndEffectorPose(const JointManifoldState& state,
                              fcl::Transform3d& transform) const override;

  [[nodiscard]] std::vector<std::shared_ptr<fcl::CollisionGeometryd>>
  getCollisionGeometries() const override;

  [[nodiscard]] bool supportsInverseKinematics(TaskSpaceType taskSpaceType) const override
  {
    return taskSpaceType == TaskSpaceType::SE2;
  }

  [[nodiscard]] bool computeInverseKinematics(const std::vector<double>& targetWorkspace,
                                              const JointManifoldState& seedState,
                                              JointManifoldState& solutionState) const override;

  [[nodiscard]] KinematicParameters getKinematicParameters() const override;

  [[nodiscard]] JointManifoldState interpolateManifoldState(const JointManifoldState& a,
                                                            const JointManifoldState& b,
                                                            double t) const override;
  [[nodiscard]] double computeManifoldDistance(const JointManifoldState& a,
                                               const JointManifoldState& b) const override;

private:
  double l1_;
  double l2_;
  double l3_;

  std::shared_ptr<fcl::CollisionGeometryd> link1Geometry_;
  std::shared_ptr<fcl::CollisionGeometryd> link2Geometry_;
  std::shared_ptr<fcl::CollisionGeometryd> link3Geometry_;
};

}  // namespace motion_planning_examples
