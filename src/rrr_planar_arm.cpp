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

TaskSpaceCoordinates RRRPlanarArm::computeTaskSpaceCoordinates(const JointManifoldState &state) const
{
    fcl::Transform3d transform = fcl::Transform3d::Identity();
    computeEndEffectorPose(state, transform);
    return SE2Coordinates{
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

    const SO2Coordinates joint1 = {state[0], state[1]};
    const SO2Coordinates joint2 = {state[2], state[3]};
    const SO2Coordinates joint3 = {state[4], state[5]};

    // Explicit group-chain composition: T03 = T01 * T12 * T23.
    const fcl::Transform3d T01Center = createPlanarRevoluteTransform(joint1, 0.5 * l1_);
    const fcl::Transform3d T01Tip = createPlanarRevoluteTransform(joint1, l1_);
    const fcl::Transform3d T12Center = createPlanarRevoluteTransform(joint2, 0.5 * l2_);
    const fcl::Transform3d T12Tip = createPlanarRevoluteTransform(joint2, l2_);
    const fcl::Transform3d T23Center = createPlanarRevoluteTransform(joint3, 0.5 * l3_);

    const fcl::Transform3d T02Center = T01Tip * T12Center;
    const fcl::Transform3d T02Tip = T01Tip * T12Tip;
    const fcl::Transform3d T03Center = T02Tip * T23Center;

    transforms.clear();
    transforms.push_back(T01Center);
    transforms.push_back(T02Center);
    transforms.push_back(T03Center);
}

void RRRPlanarArm::computeEndEffectorPose(const JointManifoldState &state,
                                          fcl::Transform3d &transform) const
{
    if (state.size() < 6) { transform = fcl::Transform3d::Identity(); return; }

    const SO2Coordinates joint1 = {state[0], state[1]};
    const SO2Coordinates joint2 = {state[2], state[3]};
    const SO2Coordinates joint3 = {state[4], state[5]};

    const fcl::Transform3d T01Tip = createPlanarRevoluteTransform(joint1, l1_);
    const fcl::Transform3d T12Tip = createPlanarRevoluteTransform(joint2, l2_);
    const fcl::Transform3d T23Tip = createPlanarRevoluteTransform(joint3, l3_);
    transform = T01Tip * T12Tip * T23Tip;
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

KinematicParameters RRRPlanarArm::getKinematicParameters() const
{
    return KinematicParameters{{l1_, l2_, l3_}};
}

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