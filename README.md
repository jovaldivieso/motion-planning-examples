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

Compact SO2 note (Interpolation): for unit states $a,b\in SO(2)$ with

$$
\omega = \mathrm{arccos}(\mathrm{clamp}(a\cdot b,-1,1)),
$$

the geodesic interpolation used by `interpolateSO2` is

$$
\mathrm{interp}(a,b,t)=
\frac{\sin((1-t)\omega)}{\sin\omega}a + \frac{\sin(t\omega)}{\sin\omega}b,
\quad t\in[0,1],
$$

followed by normalization.

Compact SO2 note (Distance): the geodesic distance used by `computeGeodesicDistanceSO2` is

$$
d_{SO(2)}(a,b)=\mathrm{arccos}(\mathrm{clamp}(a\cdot b,-1,1)).
$$

This is the shortest arc length on the unit circle between the two rotation states.

## Kinematics

### RR Planar Arm

For the 2-link arm, the end-effector position is

$$
x = l_1\cos\theta_1 + l_2\cos(\theta_1 + \theta_2), \qquad
y = l_1\sin\theta_1 + l_2\sin(\theta_1 + \theta_2).
$$

Explicit Lie-group chain form (used in code): with joint states in $SO(2)$ and homogeneous transforms in $SE(2)$,

$$
T_{0e} = T_{01}(q_1)\,T_{12}(q_2),
$$

where each factor is a planar revolute transform

$$
T(q_i,\ell)=
\begin{bmatrix}
c_i & -s_i & \ell c_i \\
s_i & c_i  & \ell s_i \\
0   & 0    & 1
\end{bmatrix},\quad q_i=[c_i,s_i]\in SO(2).
$$

For link collision geometry centers we use the same chain with $\ell=l_i/2$ for the corresponding link.

Inverse kinematics uses the standard two-branch planar solution, but the backend works directly in `SO2Coordinates` rather than reconstructing intermediate scalar angles.

Compact Lie-group derivation of the `val` term used in code:

$$
p = l_1 R_1 e_1 + l_2 R_1 R_2 e_1, \quad R_1,R_2\in SO(2), \quad e_1 = [1,0]^T.
$$

Left-multiply by $R_1^T$:

$$
u := R_1^T p = l_1 e_1 + l_2 R_2 e_1.
$$

Use norm invariance of rotations and expand:

$$
\|p\|^2 = \|u\|^2 = l_1^2 + l_2^2 + 2 l_1 l_2\, e_1^T R_2 e_1.
$$

Define $c_2 := e_1^T R_2 e_1$, then

$$
c_2 = \frac{\|p\|^2 - l_1^2 - l_2^2}{2 l_1 l_2}.
$$

This is exactly the implementation value `val`. It is a group-invariant algebraic identity (not an inverse-trigonometric derivation).

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

Theorem (Orthogonal/Parallel Decomposition): for any unit direction vector $d$ and displacement $\Delta$, we have

$$
\Delta = (dd^T)\Delta + (I - dd^T)\Delta,
$$

where $(dd^T)\Delta$ is the component parallel to $d$ and $(I - dd^T)\Delta$ is the orthogonal component. The straight-line residual uses

$$
r = w(I - dd^T)(x - x_0),
$$

so minimizing $\|r\|^2$ drives only orthogonal deviation to zero while allowing motion along the line.

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
