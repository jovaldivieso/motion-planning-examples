#pragma once

#include "manifold_types.hpp"

#include <memory>
#include <vector>

#include <fcl/fcl.h>

namespace motion_planning_examples
{

class RobotMechanism
{
public:
    virtual ~RobotMechanism() = default;

    [[nodiscard]] virtual std::size_t getJointCount() const = 0;

    virtual void computeForwardKinematicsFromManifoldState(const JointManifoldState &state, std::vector<fcl::Transform3d> &transforms) const = 0;

    virtual void computeEndEffectorFromManifoldState(const JointManifoldState &state, fcl::Transform3d &transform) const = 0;

    [[nodiscard]] virtual std::vector<std::shared_ptr<fcl::CollisionGeometryd>> getCollisionGeometries() const = 0;

    [[nodiscard]] virtual bool computeInverseKinematics(const std::vector<double>& targetWorkspace, 
                                                        const JointManifoldState &seedState,
                                                        JointManifoldState &solutionState) const = 0;

    [[nodiscard]] virtual std::vector<double> getKinematicParameters() const = 0;

    // Native continuous interpolation mechanism on the mechanism manifold.
    [[nodiscard]] virtual JointManifoldState interpolateManifoldState(const JointManifoldState& a, 
                                                                      const JointManifoldState& b, 
                                                                      double t) const = 0;

    // Native distance metric for trajectory path length calculations.
    [[nodiscard]] virtual double computeManifoldDistance(const JointManifoldState& a, 
                                                         const JointManifoldState& b) const = 0;
};

}  // namespace motion_planning_examples
