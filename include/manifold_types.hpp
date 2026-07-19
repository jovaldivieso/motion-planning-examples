#pragma once

#include <vector>

namespace motion_planning_examples
{

// A generic flat vector to hold manifold-native coordinates.
// For a 2-link planar arm: [c1, s1, c2, s2]
// For future 3D arms: could hold [w, x, y, z] quaternions or [d] prismatic distances.
using JointManifoldState = std::vector<double>;
using ManifoldPath = std::vector<JointManifoldState>;

}  // namespace motion_planning_examples
