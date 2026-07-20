# C++ Motion Planning Examples

This repository keeps the implementation and the math aligned around manifold-native robot states.

## Quickstart

```bash
cmake -S . -B build
cmake --build build -j
./build/plan rr_planar_arm_config.yaml
python plot_c_space.py --config rr_planar_arm_config.yaml
python plot_task_space.py --config rr_planar_arm_config.yaml
```

The runtime config chooses the planner and robot:

```yaml
planner: ompl   # or ceres
robot: rr_planar_arm  # or rrr_planar_arm
```

## Core Conventions

The backend does not use raw joint angles as its internal state. Revolute joints are represented as unit-circle coordinates and task spaces use explicit coordinate types.

- Joint state for a revolute joint: `SO2Coordinates = [c, s]`
- Current task-space coordinate types: `Euclidean2DCoordinates`, `Euclidean3DCoordinates`, `SE2Coordinates`, `SE3Coordinates`
- Current function naming convention: verbs first, then the manifold suffix, for example `createFromAngleSO2`, `convertToAngleSO2`, `interpolateSO2`, and `computeGeodesicDistanceSO2`
- `taskSpaceTypeName` returns capitalized task-space labels such as `SE2` and `SE3`

Scalar angles are only used at boundaries where they are unavoidable, such as config parsing, CSV export, and OMPL's internal `SO2` state assignment.

## Robot Mechanisms

The base abstraction is `RobotMechanism`, which exposes manifold-native joint states, task-space coordinates, and optional task-space-specific inverse kinematics.

### Current Mechanisms

The repository currently implements two planar robots:

- `RRPlanarArm`: 2-DOF revolute arm with Euclidean 2D task space
- `RRRPlanarArm`: 3-DOF revolute arm with `SE2` task space represented as `[x, y, c_phi, s_phi]`

Both mechanisms store each joint as `SO2Coordinates` and interpolate on the circle using `interpolateSO2`.

### State Representation

Joint configuration is stored as a flat sequence of circle coordinates:

$$
q = [c_1, s_1, c_2, s_2, \dots]
$$

This avoids angle wrapping in the planner, collision checker, and IK selection logic.

## Kinematics

### RR Planar Arm

For the 2-link arm, the end-effector position is

$$
x = l_1\cos\theta_1 + l_2\cos(\theta_1 + \theta_2), \qquad
y = l_1\sin\theta_1 + l_2\sin(\theta_1 + \theta_2).
$$

Inverse kinematics uses the standard two-branch planar solution, but the backend works directly in `SO2Coordinates` rather than reconstructing intermediate scalar angles.

### RRR Planar Arm

The 3-link arm uses a planar `SE2` task space with coordinates `[x, y, c_phi, s_phi]`.

The IK implementation reduces the problem to the wrist point, solves the first two joints from the wrist location, and then composes the final orientation in `SO2Coordinates`.

### Boundary Conversions

The current code keeps angle conversion isolated at the edges:

- `createFromAngleSO2(angle)` converts a scalar angle to `[cos(angle), sin(angle)]`
- `convertToAngleSO2([c, s])` converts back with `atan2`

These helpers are used for OMPL integration and CSV export, not for the main backend math.

## Planners

### OMPL RRT*

The OMPL example builds a compound state space with one `SO2` subspace per joint.

- Sampling and interpolation remain manifold-aware through OMPL's `SO2` spaces
- Validity is delegated to the FCL collision checker callback
- Solution paths are simplified and then interpolated

### Ceres Straight-Line Planner

The Ceres example enforces a straight-line end-effector path in task space.

Start and goal task-space positions are computed by FK, then the solver minimizes orthogonal distance to the line segment while staying on the joint manifold.

The planner asks `RobotMechanism` whether inverse kinematics is available for the requested `TaskSpaceType`, and only seeds from IK when that capability exists.

## Collision Checker

The collision checker uses FCL with static square obstacles.

1. Obstacles are modeled as FCL boxes with pose `(c_x, c_y, 0)`.
2. Robot FK returns world transforms for each link collision box.
3. Each link-obstacle pair is tested with `collide`.
4. A state is valid only if no pair collides.

The current implementation does not include self-collision between links.

## TODOs

- [ ] Add link-link self-collision checks.
- [ ] Add soft obstacle costs to the Ceres planner.
- [ ] Add a higher-DOF mechanism and redundancy objectives.
