#pragma once

#include <memory>
#include <vector>

#include <fcl/fcl.h>
#include <ompl/base/StateSpace.h>

namespace motion_planning_examples
{

class RobotMechanism
{
public:
    virtual ~RobotMechanism() = default;

    // Defines the topology of the configuration space
    [[nodiscard]] virtual std::shared_ptr<ompl::base::StateSpace> getStateSpace() const = 0;

    // Evaluates N-Link forward kinematics and fills the transforms array (Centers of Geometry for Collision)
    virtual void computeForwardKinematics(const ompl::base::State* state, 
                                          std::vector<fcl::Transform3d>& transforms) const = 0;

    // Evaluates the task space pose of the End-Effector / Tool Center Point
    virtual void computeEndEffectorTransform(const ompl::base::State* state, 
                                             fcl::Transform3d& transform) const = 0;

    // Provides the geometric shapes attached to each link for collision checking
    [[nodiscard]] virtual std::vector<std::shared_ptr<fcl::CollisionGeometryd>> getCollisionGeometries() const = 0;

    // Generic interface to request an Inverse Kinematics seed for trajectory optimizers
    [[nodiscard]] virtual bool computeInverseKinematics(const std::vector<double>& targetWorkspace, 
                                                        bool hint, 
                                                        std::vector<double>& seedState) const = 0;

    // Extracts numeric kinematic parameters (e.g. link lengths) required by analytic AutoDiff functors
    [[nodiscard]] virtual std::vector<double> getKinematicParameters() const = 0;
};

}  // namespace motion_planning_examples
