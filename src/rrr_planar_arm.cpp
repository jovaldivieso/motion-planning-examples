#include "rrr_planar_arm.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace motion_planning_examples
{

RRRPlanarArm::RRRPlanarArm(double link1Length, double link2Length, double link3Length, double linkThickness, double objectHeight)
    : l1_(link1Length), l2_(link2Length), l3_(link3Length)
{
    link1Geometry_ = std::make_shared<fcl::Boxd>(l1_, linkThickness, objectHeight);
    link2Geometry_ = std::make_shared<fcl::Boxd>(l2_, linkThickness, objectHeight);
    link3Geometry_ = std::make_shared<fcl::Boxd>(l3_, linkThickness, objectHeight);
}

std::size_t RRRPlanarArm::getJointCount() const { return 3; }

std::vector<double> RRRPlanarArm::computeTaskSpaceCoordinates(const JointManifoldState &state) const
{
    fcl::Transform3d transform = fcl::Transform3d::Identity();
    computeEndEffectorPose(state, transform);
    return {
        transform.translation().x(),
        transform.translation().y(),
        transform.linear()(0, 0),
        transform.linear()(1, 0),
    };
}

void RRRPlanarArm::computeForwardKinematics(const JointManifoldState &state,
                                            std::vector<fcl::Transform3d> &transforms) const
{
    if (state.size() < 6) { transforms.clear(); return; }

    const double c1 = state[0];
    const double s1 = state[1];
    const double c2 = state[2];
    const double s2 = state[3];
    const double c3 = state[4];
    const double s3 = state[5];

    const double c12 = c1 * c2 - s1 * s2;
    const double s12 = s1 * c2 + c1 * s2;
    const double c123 = c12 * c3 - s12 * s3;
    const double s123 = s12 * c3 + c12 * s3;

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

    fcl::Transform3d tf3 = fcl::Transform3d::Identity();
    tf3.linear()(0, 0) = c123;
    tf3.linear()(0, 1) = -s123;
    tf3.linear()(1, 0) = s123;
    tf3.linear()(1, 1) = c123;
    tf3.translation() << l1_ * c1 + l2_ * c12 + 0.5 * l3_ * c123,
                         l1_ * s1 + l2_ * s12 + 0.5 * l3_ * s123, 0.0;

    transforms.clear();
    transforms.push_back(tf1);
    transforms.push_back(tf2);
    transforms.push_back(tf3);
}

void RRRPlanarArm::computeEndEffectorPose(const JointManifoldState &state,
                                          fcl::Transform3d &transform) const
{
    if (state.size() < 6) { transform = fcl::Transform3d::Identity(); return; }

    const double c1 = state[0];
    const double s1 = state[1];
    const double c2 = state[2];
    const double s2 = state[3];
    const double c3 = state[4];
    const double s3 = state[5];

    const double c12 = c1 * c2 - s1 * s2;
    const double s12 = s1 * c2 + c1 * s2;
    const double c123 = c12 * c3 - s12 * s3;
    const double s123 = s12 * c3 + c12 * s3;

    transform = fcl::Transform3d::Identity();
    transform.linear()(0, 0) = c123;
    transform.linear()(0, 1) = -s123;
    transform.linear()(1, 0) = s123;
    transform.linear()(1, 1) = c123;
    transform.translation() << l1_ * c1 + l2_ * c12 + l3_ * c123,
                               l1_ * s1 + l2_ * s12 + l3_ * s123, 0.0;
}

std::vector<std::shared_ptr<fcl::CollisionGeometryd>> RRRPlanarArm::getCollisionGeometries() const
{
    return {link1Geometry_, link2Geometry_, link3Geometry_};
}

bool RRRPlanarArm::computeInverseKinematics(const std::vector<double>& targetWorkspace,
                                            const JointManifoldState &seedState,
                                            JointManifoldState &solutionState) const
{
    if (targetWorkspace.size() < 4 || seedState.size() < 6) return false;

    constexpr double kEps = 1e-12;

    const double px = targetWorkspace[0];
    const double py = targetWorkspace[1];
    double cPhi = targetWorkspace[2];
    double sPhi = targetWorkspace[3];
    const double phiNorm = std::sqrt(cPhi * cPhi + sPhi * sPhi);
    if (phiNorm < kEps) return false;
    cPhi /= phiNorm;
    sPhi /= phiNorm;

    const double wristX = px - l3_ * cPhi;
    const double wristY = py - l3_ * sPhi;

    const double wristSq = wristX * wristX + wristY * wristY;
    const double val = (wristSq - l1_ * l1_ - l2_ * l2_) / (2.0 * l1_ * l2_);

    if (val < -1.0 || val > 1.0) return false;

    const double c2 = val;
    const double s2Mag = std::sqrt(std::max(0.0, 1.0 - c2 * c2));

    std::array<double, 2> joint2A = {c2, s2Mag};
    std::array<double, 2> joint2B = {c2, -s2Mag};

    auto solveJoint1 = [&](const std::array<double, 2>& j2) -> std::array<double, 2> {
        if (wristSq < kEps)
        {
            const double n = std::hypot(seedState[0], seedState[1]);
            if (n < kEps) return {1.0, 0.0};
            return {seedState[0] / n, seedState[1] / n};
        }

        const double k1 = l1_ + l2_ * j2[0];
        const double k2 = l2_ * j2[1];

        const double c1 = (k1 * wristX + k2 * wristY) / wristSq;
        const double s1 = (-k2 * wristX + k1 * wristY) / wristSq;
        return {c1, s1};
    };

    auto j1A = solveJoint1(joint2A);
    auto j1B = solveJoint1(joint2B);

    const double c12A = j1A[0] * joint2A[0] - j1A[1] * joint2A[1];
    const double s12A = j1A[1] * joint2A[0] + j1A[0] * joint2A[1];
    const double c3A = c12A * cPhi + s12A * sPhi;
    const double s3A = -s12A * cPhi + c12A * sPhi;

    const double c12B = j1B[0] * joint2B[0] - j1B[1] * joint2B[1];
    const double s12B = j1B[1] * joint2B[0] + j1B[0] * joint2B[1];
    const double c3B = c12B * cPhi + s12B * sPhi;
    const double s3B = -s12B * cPhi + c12B * sPhi;

    const JointManifoldState candidateA = {
        j1A[0], j1A[1],
        joint2A[0], joint2A[1],
        c3A, s3A
    };
    const JointManifoldState candidateB = {
        j1B[0], j1B[1],
        joint2B[0], joint2B[1],
        c3B, s3B
    };

    if (computeManifoldDistance(candidateA, seedState) <= computeManifoldDistance(candidateB, seedState))
    {
        solutionState = candidateA;
    }
    else
    {
        solutionState = candidateB;
    }
    return true;
}

std::vector<double> RRRPlanarArm::getKinematicParameters() const { return {l1_, l2_, l3_}; }

JointManifoldState RRRPlanarArm::interpolateManifoldState(const JointManifoldState& a, const JointManifoldState& b, double t) const
{
    const SO2Coordinates j1 = interpolateSO2({a[0], a[1]}, {b[0], b[1]}, t);
    const SO2Coordinates j2 = interpolateSO2({a[2], a[3]}, {b[2], b[3]}, t);
    const SO2Coordinates j3 = interpolateSO2({a[4], a[5]}, {b[4], b[5]}, t);
    return {j1[0], j1[1], j2[0], j2[1], j3[0], j3[1]};
}

double RRRPlanarArm::computeManifoldDistance(const JointManifoldState& a, const JointManifoldState& b) const
{
    const double d1 = computeGeodesicDistanceSO2({a[0], a[1]}, {b[0], b[1]});
    const double d2 = computeGeodesicDistanceSO2({a[2], a[3]}, {b[2], b[3]});
    const double d3 = computeGeodesicDistanceSO2({a[4], a[5]}, {b[4], b[5]});
    return std::sqrt(d1 * d1 + d2 * d2 + d3 * d3);
}

}  // namespace motion_planning_examples