#include "two_dof_planar_arm.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace motion_planning_examples
{

TwoDOFPlanarArm::TwoDOFPlanarArm(double link1Length, double link2Length, double linkThickness, double objectHeight)
    : l1_(link1Length), l2_(link2Length)
{
    link1Geometry_ = std::make_shared<fcl::Boxd>(l1_, linkThickness, objectHeight);
    link2Geometry_ = std::make_shared<fcl::Boxd>(l2_, linkThickness, objectHeight);
}

std::size_t TwoDOFPlanarArm::getJointCount() const { return 2; }

void TwoDOFPlanarArm::computeForwardKinematics(const JointManifoldState &state,
                                               std::vector<fcl::Transform3d> &transforms) const
{
    if (state.size() < 4) { transforms.clear(); return; }

    const double c1 = state[0];
    const double s1 = state[1];
    const double c12 = c1 * state[2] - s1 * state[3];
    const double s12 = s1 * state[2] + c1 * state[3];

    fcl::Transform3d tf1 = fcl::Transform3d::Identity();
    tf1.linear()(0, 0) = c1;
    tf1.linear()(0, 1) = -s1;
    tf1.linear()(1, 0) = s1;
    tf1.linear()(1, 1) = c1;
    tf1.translation() << 0.5 * l1_ * c1, 0.5 * l1_ * s1, 0.0;

    fcl::Transform3d tf2 = fcl::Transform3d::Identity();
    tf2.linear()(0, 0) = c12;
    tf2.linear()(0, 1) = -s12;
    tf2.linear()(1, 0) = s12;
    tf2.linear()(1, 1) = c12;
    tf2.translation() << l1_ * c1 + 0.5 * l2_ * c12,
                         l1_ * s1 + 0.5 * l2_ * s12, 0.0;

    transforms.clear();
    transforms.push_back(tf1);
    transforms.push_back(tf2);
}

void TwoDOFPlanarArm::computeEndEffectorPose(const JointManifoldState &state,
                                         fcl::Transform3d &transform) const
{
    if (state.size() < 4) { transform = fcl::Transform3d::Identity(); return; }

    const double c1 = state[0];
    const double s1 = state[1];
    const double c12 = c1 * state[2] - s1 * state[3];
    const double s12 = s1 * state[2] + c1 * state[3];

    transform = fcl::Transform3d::Identity();
    transform.linear()(0, 0) = c12;
    transform.linear()(0, 1) = -s12;
    transform.linear()(1, 0) = s12;
    transform.linear()(1, 1) = c12;
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
    if (targetWorkspace.size() < 2 || seedState.size() < 4) return false;
    double px = targetWorkspace[0];
    double py = targetWorkspace[1];

    double p_sq = px * px + py * py;
    constexpr double kEps = 1e-12;
    double val = (p_sq - l1_ * l1_ - l2_ * l2_) / (2.0 * l1_ * l2_);
    
    if (val < -1.0 || val > 1.0) return false; // Unreachable

    // 1. Solve for Joint 2 algebraically
    double c2 = val; 
    double s2_mag = std::sqrt(1.0 - c2 * c2);
    
    std::array<double, 2> joint2_a = {c2, s2_mag};
    std::array<double, 2> joint2_b = {c2, -s2_mag};

    // 2. Solve for Joint 1 as a 2x2 linear system (No trigonometry)
    auto solveJoint1 = [&](const std::array<double, 2>& j2) -> std::array<double, 2> {
        if (p_sq < kEps)
        {
            // At the origin singularity (e.g. l1 == l2 and elbow folded), theta1 is underdetermined.
            // Reuse seed direction to preserve branch continuity.
            const double n = std::hypot(seedState[0], seedState[1]);
            if (n < kEps) return {1.0, 0.0};
            return {seedState[0] / n, seedState[1] / n};
        }

        double k1 = l1_ + l2_ * j2[0];
        double k2 = l2_ * j2[1];
        
        // p_sq is the determinant of the k1, k2 matrix
        double c1 = (k1 * px + k2 * py) / p_sq;
        double s1 = (-k2 * px + k1 * py) / p_sq;
        return {c1, s1};
    };

    auto j1_a = solveJoint1(joint2_a);
    auto j1_b = solveJoint1(joint2_b);

    const JointManifoldState candidateA = {j1_a[0], j1_a[1], joint2_a[0], joint2_a[1]};
    const JointManifoldState candidateB = {j1_b[0], j1_b[1], joint2_b[0], joint2_b[1]};

    if (computeManifoldDistance(candidateA, seedState) <= computeManifoldDistance(candidateB, seedState)) {
        solutionState = candidateA;
    } else {
        solutionState = candidateB;
    }
    return true;
}

std::vector<double> TwoDOFPlanarArm::getKinematicParameters() const { return {l1_, l2_}; }

JointManifoldState TwoDOFPlanarArm::interpolateManifoldState(const JointManifoldState& a, const JointManifoldState& b, double t) const
{
    auto slerp = [t](double a0, double a1, double b0, double b1) -> std::array<double, 2> {
        double dot = std::clamp(a0 * b0 + a1 * b1, -1.0, 1.0);
        double omega = std::acos(dot);
        if (omega < 1e-10) return {a0, a1};
        
        double sinOmega = std::sin(omega);
        double wa = std::sin((1.0 - t) * omega) / sinOmega;
        double wb = std::sin(t * omega) / sinOmega;
        
        double out0 = wa * a0 + wb * b0;
        double out1 = wa * a1 + wb * b1;
        double n = std::sqrt(out0 * out0 + out1 * out1);
        return (n < 1e-12) ? std::array<double, 2>{a0, a1} : std::array<double, 2>{out0 / n, out1 / n};
    };

    auto j1 = slerp(a[0], a[1], b[0], b[1]);
    auto j2 = slerp(a[2], a[3], b[2], b[3]);
    return {j1[0], j1[1], j2[0], j2[1]};
}

double TwoDOFPlanarArm::computeManifoldDistance(const JointManifoldState& a, const JointManifoldState& b) const
{
    const double dot1 = std::clamp(a[0] * b[0] + a[1] * b[1], -1.0, 1.0);
    const double dot2 = std::clamp(a[2] * b[2] + a[3] * b[3], -1.0, 1.0);
    const double d1 = std::acos(dot1);
    const double d2 = std::acos(dot2);
    return std::sqrt(d1 * d1 + d2 * d2);
}

}  // namespace motion_planning_examples
