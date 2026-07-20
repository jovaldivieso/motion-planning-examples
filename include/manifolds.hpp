#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

#include <fcl/fcl.h>

namespace motion_planning_examples
{

using JointManifoldState = std::vector<double>;
using ManifoldPath = std::vector<JointManifoldState>;

using Euclidean2DCoordinates = std::array<double, 2>;
using Euclidean3DCoordinates = std::array<double, 3>;
using SO2Coordinates = std::array<double, 2>;   // [c, s]
using SO3Coordinates = std::array<double, 3>;   // rotation vector coordinates
using SE2Coordinates = std::array<double, 4>;   // [x, y, c_phi, s_phi]
using SE3Coordinates = std::array<double, 16>;  // row-major 4x4 homogeneous transform
using TaskSpaceCoordinates =
    std::variant<Euclidean2DCoordinates, Euclidean3DCoordinates, SE2Coordinates, SE3Coordinates>;

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
    case TaskSpaceType::Euclidean2D:
      return 2;
    case TaskSpaceType::Euclidean3D:
      return 3;
    case TaskSpaceType::SE2:
      return 4;
    case TaskSpaceType::SE3:
      return 16;
  }
  return 0;
}

[[nodiscard]] inline constexpr std::string_view getTaskSpaceTypeName(TaskSpaceType type)
{
  switch (type)
  {
    case TaskSpaceType::Euclidean2D:
      return "euclidean2d";
    case TaskSpaceType::Euclidean3D:
      return "euclidean3d";
    case TaskSpaceType::SE2:
      return "SE2";
    case TaskSpaceType::SE3:
      return "SE3";
  }
  return "unknown";
}

[[nodiscard]] inline constexpr bool hasTaskSpaceOrientation(TaskSpaceType type)
{
  return type == TaskSpaceType::SE2 || type == TaskSpaceType::SE3;
}

[[nodiscard]] inline TaskSpaceType getTaskSpaceType(const TaskSpaceCoordinates& coordinates)
{
  return std::visit(
      [](const auto& value) -> TaskSpaceType {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, Euclidean2DCoordinates>)
          return TaskSpaceType::Euclidean2D;
        if constexpr (std::is_same_v<T, Euclidean3DCoordinates>)
          return TaskSpaceType::Euclidean3D;
        if constexpr (std::is_same_v<T, SE2Coordinates>)
          return TaskSpaceType::SE2;
        return TaskSpaceType::SE3;
      },
      coordinates);
}

[[nodiscard]] inline std::vector<double>
flattenTaskSpaceCoordinates(const TaskSpaceCoordinates& coordinates)
{
  return std::visit(
      [](const auto& value) { return std::vector<double>(value.begin(), value.end()); },
      coordinates);
}

[[nodiscard]] inline constexpr SO2Coordinates createFromAngleSO2(double angle)
{
  return { std::cos(angle), std::sin(angle) };
}

[[nodiscard]] inline double convertToAngleSO2(const SO2Coordinates& state)
{
  return std::atan2(state[1], state[0]);
}

[[nodiscard]] inline SO2Coordinates normalizeSO2(const SO2Coordinates& state)
{
  const double norm = std::sqrt(state[0] * state[0] + state[1] * state[1]);
  if (norm < 1e-12)
    return { 1.0, 0.0 };
  return { state[0] / norm, state[1] / norm };
}

[[nodiscard]] inline SO2Coordinates composeSO2(const SO2Coordinates& a, const SO2Coordinates& b)
{
  return { a[0] * b[0] - a[1] * b[1], a[1] * b[0] + a[0] * b[1] };
}

[[nodiscard]] inline SO2Coordinates inverseSO2(const SO2Coordinates& state)
{
  return { state[0], -state[1] };
}

[[nodiscard]] inline SO2Coordinates interpolateSO2(const SO2Coordinates& a, const SO2Coordinates& b,
                                                   double t)
{
  const double dot = std::clamp(a[0] * b[0] + a[1] * b[1], -1.0, 1.0);
  const double omega = std::acos(dot);
  if (omega < 1e-10)
    return a;

  const double sinOmega = std::sin(omega);
  const double wa = std::sin((1.0 - t) * omega) / sinOmega;
  const double wb = std::sin(t * omega) / sinOmega;
  const SO2Coordinates out = { wa * a[0] + wb * b[0], wa * a[1] + wb * b[1] };
  return normalizeSO2(out);
}

[[nodiscard]] inline double computeGeodesicDistanceSO2(const SO2Coordinates& a,
                                                       const SO2Coordinates& b)
{
  const double dot = std::clamp(a[0] * b[0] + a[1] * b[1], -1.0, 1.0);
  return std::acos(dot);
}

[[nodiscard]] inline fcl::Transform3d createPlanarRevoluteTransform(const SO2Coordinates& rotation,
                                                                    double xLocal)
{
  fcl::Transform3d transform = fcl::Transform3d::Identity();
  transform.linear()(0, 0) = rotation[0];
  transform.linear()(0, 1) = -rotation[1];
  transform.linear()(1, 0) = rotation[1];
  transform.linear()(1, 1) = rotation[0];
  transform.translation() << xLocal * rotation[0], xLocal * rotation[1], 0.0;
  return transform;
}

class TaskSpaceInterpolator
{
public:
  [[nodiscard]] TaskSpaceCoordinates interpolate(const TaskSpaceCoordinates& a,
                                                 const TaskSpaceCoordinates& b, double t) const
  {
    if (a.index() != b.index())
    {
      throw std::invalid_argument("Task-space coordinate types must match for interpolation");
    }

    return std::visit(
        [t](const auto& lhs, const auto& rhs) -> TaskSpaceCoordinates {
          using T = std::decay_t<decltype(lhs)>;
          T out{};

          if constexpr (std::is_same_v<T, SE2Coordinates>)
          {
            out[0] = (1.0 - t) * lhs[0] + t * rhs[0];
            out[1] = (1.0 - t) * lhs[1] + t * rhs[1];
            const SO2Coordinates orientation =
                interpolateSO2({ lhs[2], lhs[3] }, { rhs[2], rhs[3] }, t);
            out[2] = orientation[0];
            out[3] = orientation[1];
            return out;
          }

          for (std::size_t i = 0; i < out.size(); ++i)
          {
            out[i] = (1.0 - t) * lhs[i] + t * rhs[i];
          }
          return out;
        },
        a, b);
  }
};

}  // namespace motion_planning_examples
