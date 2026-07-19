#pragma once

#include <array>
#include <vector>

namespace motion_planning_examples
{

using JointS1 = std::array<double, 2>;                 // [cos(theta), sin(theta)]
using JointManifoldState = std::vector<JointS1>;       // one S1 element per joint
using ManifoldPath = std::vector<JointManifoldState>;  // sequence of manifold states

}  // namespace motion_planning_examples
