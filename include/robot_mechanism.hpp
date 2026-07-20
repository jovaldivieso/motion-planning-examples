#pragma once

#include "manifolds.hpp"

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

    // Planner-facing task-space coordinates. This may be lower-dimensional than the task space
    // for which a mechanism can provide analytic IK.
    [[nodiscard]] virtual std::vector<double> computeTaskSpaceCoordinates(const JointManifoldState &state) const = 0;

    virtual void computeForwardKinematics(const JointManifoldState &state,
                                          std::vector<fcl::Transform3d> &transforms) const = 0;

    virtual void computeEndEffectorPose(const JointManifoldState &state, fcl::Transform3d &transform) const = 0;

    [[nodiscard]] virtual std::vector<std::shared_ptr<fcl::CollisionGeometryd>> getCollisionGeometries() const = 0;

    // Whether the mechanism provides analytic IK for some task space.
    // Planners may still use lower-dimensional task spaces where the mechanism is redundant.
    [[nodiscard]] virtual bool supportsInverseKinematics(TaskSpaceType taskSpaceType) const
    {
        (void)taskSpaceType;
        return false;
    }

    [[nodiscard]] virtual bool computeInverseKinematics(const std::vector<double>& targetWorkspace,
                                                        const JointManifoldState &seedState,
                                                        JointManifoldState &solutionState) const
    {
        (void)targetWorkspace;
        (void)seedState;
        (void)solutionState;
        return false;
    }

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
