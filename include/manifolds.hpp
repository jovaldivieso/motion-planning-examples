#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace motion_planning_examples
{

using JointManifoldState = std::vector<double>;
using ManifoldPath = std::vector<JointManifoldState>;

using Euclidean2DCoordinates = std::array<double, 2>;
using Euclidean3DCoordinates = std::array<double, 3>;
using SO2Coordinates = std::array<double, 2>; // [c, s]
using SO3Coordinates = std::array<double, 3>; // rotation vector coordinates
using SE2Coordinates = std::array<double, 4>; // [x, y, c_phi, s_phi]
using SE3Coordinates = std::array<double, 16>; // row-major 4x4 homogeneous transform

enum class TaskSpaceType
{
    Euclidean2D,
    Euclidean3D,
    SE2,
    SE3,
};

[[nodiscard]] inline constexpr std::size_t getTaskSpaceCoordinateCount(TaskSpaceType type)
{
    switch (type)
    {
    case TaskSpaceType::Euclidean2D: return 2;
    case TaskSpaceType::Euclidean3D: return 3;
    case TaskSpaceType::SE2: return 4;
    case TaskSpaceType::SE3: return 16;
    }
    return 0;
}

[[nodiscard]] inline constexpr std::string_view getTaskSpaceTypeName(TaskSpaceType type)
{
    switch (type)
    {
    case TaskSpaceType::Euclidean2D: return "euclidean2d";
    case TaskSpaceType::Euclidean3D: return "euclidean3d";
    case TaskSpaceType::SE2: return "SE2";
    case TaskSpaceType::SE3: return "SE3";
    }
    return "unknown";
}

[[nodiscard]] inline constexpr TaskSpaceType inferTaskSpaceType(std::size_t coordinateCount)
{
    switch (coordinateCount)
    {
    case 2: return TaskSpaceType::Euclidean2D;
    case 3: return TaskSpaceType::Euclidean3D;
    case 4: return TaskSpaceType::SE2;
    case 16: return TaskSpaceType::SE3;
    default:
        throw std::invalid_argument("Unsupported task-space coordinate count");
    }
}

[[nodiscard]] inline constexpr bool hasTaskSpaceOrientation(TaskSpaceType type)
{
    return type == TaskSpaceType::SE2 || type == TaskSpaceType::SE3;
}

[[nodiscard]] inline constexpr SO2Coordinates createFromAngleSO2(double angle)
{
    return {std::cos(angle), std::sin(angle)};
}

[[nodiscard]] inline double convertToAngleSO2(const SO2Coordinates &state)
{
    return std::atan2(state[1], state[0]);
}

[[nodiscard]] inline SO2Coordinates normalizeSO2(const SO2Coordinates &state)
{
    const double norm = std::sqrt(state[0] * state[0] + state[1] * state[1]);
    if (norm < 1e-12) return {1.0, 0.0};
    return {state[0] / norm, state[1] / norm};
}

[[nodiscard]] inline SO2Coordinates composeSO2(const SO2Coordinates &a, const SO2Coordinates &b)
{
    return {a[0] * b[0] - a[1] * b[1], a[1] * b[0] + a[0] * b[1]};
}

[[nodiscard]] inline SO2Coordinates inverseSO2(const SO2Coordinates &state)
{
    return {state[0], -state[1]};
}

[[nodiscard]] inline SO2Coordinates interpolateSO2(const SO2Coordinates &a, const SO2Coordinates &b, double t)
{
    const double dot = std::clamp(a[0] * b[0] + a[1] * b[1], -1.0, 1.0);
    const double omega = std::acos(dot);
    if (omega < 1e-10) return a;

    const double sinOmega = std::sin(omega);
    const double wa = std::sin((1.0 - t) * omega) / sinOmega;
    const double wb = std::sin(t * omega) / sinOmega;
    const SO2Coordinates out = {wa * a[0] + wb * b[0], wa * a[1] + wb * b[1]};
    return normalizeSO2(out);
}

[[nodiscard]] inline double computeGeodesicDistanceSO2(const SO2Coordinates &a, const SO2Coordinates &b)
{
    const double dot = std::clamp(a[0] * b[0] + a[1] * b[1], -1.0, 1.0);
    return std::acos(dot);
}

[[nodiscard]] inline std::vector<double> interpolateTaskSpaceCoordinates(TaskSpaceType type,
                                                                          const std::vector<double> &a,
                                                                          const std::vector<double> &b,
                                                                          double t)
{
    if (a.size() != b.size())
    {
        throw std::invalid_argument("Task-space coordinate dimensions must match");
    }

    std::vector<double> out(getTaskSpaceCoordinateCount(type), 0.0);
    if (type == TaskSpaceType::SE2)
    {
        if (a.size() != 4) throw std::invalid_argument("SE2 task space expects 4 coordinates");
        out[0] = (1.0 - t) * a[0] + t * b[0];
        out[1] = (1.0 - t) * a[1] + t * b[1];
        const SO2Coordinates wa = interpolateSO2({a[2], a[3]}, {b[2], b[3]}, t);
        out[2] = wa[0];
        out[3] = wa[1];
        return out;
    }

    if (a.size() != out.size())
    {
        throw std::invalid_argument("Task-space coordinate dimensions do not match the requested type");
    }

    for (std::size_t i = 0; i < out.size(); ++i)
    {
        out[i] = (1.0 - t) * a[i] + t * b[i];
    }
    return out;
}

}  // namespace motion_planning_examples