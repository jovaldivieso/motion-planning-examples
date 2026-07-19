# C++ Motion Planning Examples

Repository notes to keep theory and implementation aligned, and details about the math in the implementation.

## Quickstart

```bash
cmake -S . -B build
cmake --build build -j
./build/plan config.yaml
python plot_c_space.py --config config.yaml
python plot_task_space.py --config config.yaml
```

Planner selection is in `config.yaml`:

```yaml
planner: ompl   # or ceres
```

## Robot Mechanisms

The base abstraction is `RobotMechanism`, intended for multiple mechanisms (not only the current 2-link planar arm example).

### Mechanism Specification

Current implemented mechanism instance is a 2-link planar revolute arm:

- Joint coordinates: $q = (\theta_1, \theta_2) \in \mathcal{Q}$
- Configuration manifold: $\mathcal{Q} = S^1 \times S^1 = T^2$
- Link lengths: $l_1, l_2$
- Collision geometry per link: 3D boxes in FCL (length = link length, thickness from config, and finite height)

In code, this is represented with manifold states on $S^1$ per joint, and OMPL uses `SO2` state spaces for the same topology.

### Forward Kinematics (FK)

For end-effector position in the plane:

$$
x = l_1\cos\theta_1 + l_2\cos(\theta_1 + \theta_2), \qquad
y = l_1\sin\theta_1 + l_2\sin(\theta_1 + \theta_2).
$$

For collision, each link transform is evaluated at the center of its box geometry:

- Link 1 center: $\left(\frac{l_1}{2}\cos\theta_1,\ \frac{l_1}{2}\sin\theta_1\right)$
- Link 2 center:

$$
\left(l_1\cos\theta_1 + \frac{l_2}{2}\cos(\theta_1+\theta_2),\
l_1\sin\theta_1 + \frac{l_2}{2}\sin(\theta_1+\theta_2)\right)
$$

Rotations are about world $z$ using $R_z(\theta)$, and the end-effector orientation is $R_z(\theta_1+\theta_2)$.

### FK with Matrix Lie Groups

We can write the planar chain in homogeneous form with $SE(2)$ transforms:

$$
{}^{0}T_{1}(\theta_1) =
\begin{bmatrix}
\cos\theta_1 & -\sin\theta_1 & l_1\cos\theta_1 \\
\sin\theta_1 & \cos\theta_1 & l_1\sin\theta_1 \\
0 & 0 & 1
\end{bmatrix},
\qquad
{}^{1}T_{2}(\theta_2) =
\begin{bmatrix}
\cos\theta_2 & -\sin\theta_2 & l_2\cos\theta_2 \\
\sin\theta_2 & \cos\theta_2 & l_2\sin\theta_2 \\
0 & 0 & 1
\end{bmatrix}.
$$

Then:

$$
{}^{0}T_{2}(\theta_1,\theta_2) = {}^{0}T_{1}(\theta_1)\,{}^{1}T_{2}(\theta_2).
$$

The implementation uses 3D rigid transforms (`fcl::Transform3d`) with planar motion embedded in $SE(3)$ (z-translation fixed to 0, rotation only around z).

### Geometric IK (Analytic, Manifold Output)

Given target point $(p_x,p_y)$, the code solves:

$$
c_2 = \frac{p_x^2 + p_y^2 - l_1^2 - l_2^2}{2 l_1 l_2}.
$$

If $c_2 \notin [-1,1]$, target is unreachable.

Otherwise two branches are used:

$$
θ_2^{(\pm)} = \pm\arccos(c_2),
$$

$$
θ_1^{(\pm)} = \operatorname{atan2}(p_y,p_x) -
\operatorname{atan2}\bigl(l_2\sin\theta_2^{(\pm)},\ l_1 + l_2\cos\theta_2^{(\pm)}\bigr).
$$

The branch is chosen by manifold proximity to a manifold seed state (used to keep branch continuity along the waypoint sequence).

Important implementation detail:

- IK output is manifold-native: each joint is returned as a unit vector $[\cos\theta, \sin\theta] \in S^1$.
- Backend IK/FK/planners do not apply scalar angle wrapping.
- Scalar angles are only used at boundaries where external APIs require them (for example OMPL `SO2` scalar field assignment and CSV export for plotting).

### Manifold Representation Policy

Backend state representation uses manifold coordinates directly:

- Per-joint state: unit vector $v = [\cos\theta,\sin\theta] \in S^1$.
- Robot joint configuration: Cartesian product of per-joint manifold factors.
- Path representation in planners: sequence of manifold states.

No explicit angle wrapping is used in backend trajectory optimization or IK selection.

## Global Planners

### OMPL RRT* on the Manifold

Global planner uses OMPL RRT* over manifold configuration spaces (for the current mechanism: $T^2 = S^1\times S^1$).

- State space: `CompoundStateSpace(SO2, SO2)` for current 2-joint example
- Sampling: manifold-aware via OMPL's `SO2` spaces
- Validity: delegated to FCL collision checker callback
- Post processing: OMPL `simplifySolution(1.5)` then interpolation

Path-length objective is OMPL's geometric path length under the underlying state-space metric. On this manifold, shortest paths are geodesics of that metric in obstacle-free regions; with obstacles, RRT* asymptotically approaches the optimal feasible path as samples increase.

## Local Planners

### Ceres Straight-Line End-Effector Planner

Local planner enforces a straight line in task space between start and goal end-effector points.

#### 1) Line Parameterization in Task Space

Start and goal end-effector positions are computed by FK:

$$
p_s = (x_s,y_s),\quad p_g = (x_g,y_g),\quad d = \frac{p_g-p_s}{\lVert p_g-p_s \rVert}.
$$

#### 2) Projection Cost (Distance-to-Line Residual)

The code builds a projection matrix:

$$
P = I - dd^T =
\begin{bmatrix}
1-d_x^2 & -d_x d_y \\
-d_x d_y & 1-d_y^2
\end{bmatrix}.
$$

For each waypoint end-effector point $p_i$, residual is:

$$
r_i = w\,P\,(p_i - p_s),
$$

with $w=20$ in current implementation. Minimizing this penalizes only orthogonal deviation from the line, not progress along the line.

#### 3) Manifold Optimization in Ceres

Instead of optimizing raw angles directly, each joint is represented as a unit vector:

$$
v = [\cos\theta,\ \sin\theta] \in S^1.
$$

For waypoint $i$, the solver variables are $(v_{1,i}, v_{2,i})$ for joints 1 and 2. Ceres applies `SphereManifold<2>` to each 2D parameter block, so updates stay on the unit circle manifold.

This avoids angle-wrapping artifacts during optimization and keeps a smooth manifold parameterization.

#### 4) Smoothness Term

For neighboring waypoints, code adds:

$$
r^{\text{smooth}}_{i} = \lambda (v_{i+1} - v_i), \quad \lambda = 1.0,
$$

for both joints. Start and goal waypoints are fixed as constants.

#### 5) Collision Handling (Current Behavior)

Collision validation in the backend is performed directly from manifold states through manifold-native FK. If any state collides, solve is rejected. There is no obstacle repulsive term in the Ceres objective yet.

## Collision Checker

### FCL Setup

Current collision setup checks robot links vs static square obstacles:

1. Obstacles are modeled as FCL boxes with pose $(c_x,c_y,0)$ and side length from generated map.
2. For a queried state, robot FK returns world transforms for each link collision box.
3. For each link-obstacle pair, FCL `collide` is called with `num_max_contacts = 1`.
4. State is valid iff no pair collides.

Notes:

- This currently does not include self-collision between links.
- Collision map CSV is generated by sampling a grid in $(\theta_1,\theta_2)$.

## TODO's

- [ ] Self-collisions with FCL: add link-link checks in addition to link-obstacle checks.
- [ ] Repulsive obstacle costs in Ceres: add soft obstacle terms so local optimization can bend around obstacles.
- [ ] 3-DOF mechanism and redundancy resolution: add null-space objectives (e.g., manipulability, joint motion penalties).
