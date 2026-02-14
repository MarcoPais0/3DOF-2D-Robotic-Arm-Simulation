# 4DOF 3D Robotic Arm Project Baseline

## Purpose of This Document

This file is the baseline reference for the current 4DOF arm project.

Its role is to:

- explain the mathematical model before further implementation work,
- map each mathematical concept to the code that currently exists,
- distinguish clearly between what is already implemented and what is still an intended upgrade,
- serve as the source document that can later be refined into:
  - a more formal academic report,
  - a less formal and shorter README.

This document is intentionally math-first and status-tagged. It explains the system as it exists today, records the current mathematical limits of the implemented model, and then highlights the gaps to a more realistic spatial 4DOF manipulator.

## How This Baseline Should Be Used

The intended workflow for the project is:

1. Understand the model and notation.
2. Refine one topic at a time until the math and assumptions are fully clear.
3. Update or extend the implementation only after the topic is understood.
4. Consolidate the final understanding into a structured report and a concise README.

This makes the document the canonical conceptual interface for the project. Code may change later, but the notation and interpretation defined here should remain consistent unless deliberately revised.

The status tags used throughout the baseline are:

- `Current`: implemented in code today.
- `Planned`: intended baseline target that is not yet implemented.
- `Extension`: optional follow-on analysis, reporting topic, or later refinement.

## Conceptual Flow Between Topics

The core mathematical dependency chain in the project is:

```text
Robot Geometry
      ↓
   DH Table
      ↓
Transformation Matrices
      ↓
Forward Kinematics
      ↓
Jacobian
      ↓
Inverse Kinematics
```

In this baseline, that flow maps to the topics as follows:

- Robot Geometry -> Topic 1: manipulator definition and frames
- DH Table -> Topic 2: DH parameters and the chosen pedagogical spatial example
- Transformation Matrices -> Topic 2: per-link homogeneous transforms `T_(i-1)^i`
- Forward Kinematics -> Topic 3: cumulative pose of the wrist and tool frame
- Jacobian -> Topic 5: velocity mapping from joint space to task space
- Inverse Kinematics -> Topic 6: currently handled as inverse differential kinematics through the damped pseudo-inverse, rather than as a closed-form analytical solver

The remaining topics sit around this core chain:

- Topic 4 uses forward kinematics for visualization
- Topics 7, 8, and 9 add dynamics, control, and numerical integration on top of the kinematic model
- Topic 10 packages the stack into the demo layer and reporting outputs

## Current Project State

### Current Repository Structure

The present 4DOF project is organized around the following modules:

- `src/arm4dof_dh.py`: manipulator definition, DH transform, forward kinematics, and joint positions for plotting.
- `src/jacobian4dof.py`: geometric Jacobian, damped pseudo-inverse, and inverse differential kinematics.
- `src/dynamics4dof.py`: simplified joint-space dynamics, approximate gravity, and semi-implicit Euler integration.
- `src/control4dof.py`: per-joint PID control and joint-space step response simulation.
- `src/simulation_4dof_3d.py`: PyVista-based 3D visualization and the demo layer.
- `src/pid_sandbox.py`: isolated PID intuition and tuning sandbox for a simple second-order plant.
- `Report_4DOF_3D_Robotic_Arm_Simulation.md`: current narrative summary of the implemented ideas.

### What the Current Model Actually Is

Although the project is presented as a "4DOF 3D" robotic arm simulation, the current kinematic chain is not yet a fully spatial 3D manipulator.

The present implementation uses:

- four revolute joints,
- standard DH notation,
- `alpha_i = 0` for all links,
- `d_i = 0` for all links,
- configurable link lengths `a_i = [L1, L2, L3, L4]`.

This means the mechanism is currently a planar 4R serial chain embedded and visualized in 3D space. All motion is generated in the XY plane, while the Z coordinate remains zero in the default model.

### Why This Distinction Matters

This distinction is central for interpretation:

- the visualization is 3D,
- the state dimension is 4,
- the Jacobian is written as a `6 x 4` geometric Jacobian,
- but the physical geometry is still planar because all joint axes remain parallel and there are no out-of-plane offsets or twists.

Therefore, the baseline for the project must explain the current model faithfully before proposing future upgrades to a true spatial arm.

## System Model and Notation

The notation below is the standard vocabulary for the project.

| Symbol | Dimension | Meaning |
| --- | --- | --- |
| `q` | `R^4` | Joint angle vector, `q = [q1, q2, q3, q4]^T` |
| `q_dot` | `R^4` | Joint velocity vector |
| `q_ddot` | `R^4` | Joint acceleration vector |
| `tau` | `R^4` | Joint torque vector |
| `a_i` | scalar | DH link length for link `i` |
| `alpha_i` | scalar | DH link twist for link `i` |
| `d_i` | scalar | DH link offset for link `i` |
| `theta_i` | scalar | DH joint angle for link `i` |
| `T_(i-1)^i` | `R^(4x4)` | Homogeneous transform from frame `i-1` to frame `i` |
| `T_0^4(q)` | `R^(4x4)` | Homogeneous transform of joint 4 / wrist frame with respect to the base |
| `T_4^tool` | `R^(4x4)` | Fixed homogeneous transform from joint 4 to the tool frame |
| `T_0^tool(q)` | `R^(4x4)` | Tool / end-effector homogeneous transform with respect to the base |
| `x` | `R^3` | Tool-frame Cartesian position |
| `v_desired` | `R^3` | Desired end-effector linear velocity for position control |
| `xdot_desired` | `R^6` | Embedded spatial command `[v_desired; 0]` used by the current implementation |
| `J(q)` | `R^(6x4)` | Geometric Jacobian |
| `J_v(q)` | `R^(3x4)` | Linear block of the geometric Jacobian |
| `J^+` | `R^(4x6)` | Damped least-squares pseudo-inverse of the Jacobian |
| `J_v^+` | `R^(4x3)` | Damped least-squares pseudo-inverse of the position Jacobian |
| `M(q)` | `R^(4x4)` | Joint-space inertia matrix |
| `D` | `R^(4x4)` | Joint damping matrix |
| `g(q)` | `R^4` | Gravity torque vector |
| `dt` | scalar | Time step |

### Interface Definitions by Status

The following interfaces summarize the main mathematical relationships while separating what the code implements today from what the baseline is targeting next.

- `Current`:
  - Forward kinematics: `T_0^4(q) = prod_(i=1)^4 T_(i-1)^i(q_i)`
  - Differential kinematics: `xdot = J(q) q_dot`
  - Inverse differential kinematics: `q_dot = J^+ xdot_desired`, with the current Cartesian demo using `xdot_desired = [v_x, v_y, 0, 0, 0, 0]^T`
  - Simplified dynamics: `M q_ddot = tau - D q_dot - g(q)`
  - Joint-space PID: `tau_i = Kp_i e_i + Ki_i integral(e_i dt) + Kd_i e_dot_i`
- `Planned`:
  - Tool-frame forward kinematics: `T_0^tool(q) = T_0^4(q) T_4^tool`
  - Position Jacobian interface: `v = J_v(q) q_dot`
  - Position-only inverse differential kinematics: `q_dot = J_v^+ v_desired`
- `Extension`:
  - Gravity-compensated and saturated PID: `tau_i = sat(Kp_i e_i + Ki_i eta_i + Kd_i e_dot_i + g_hat_i(q))`

## Topic 1: Manipulator Definition and Frames

### Objective

Define the robot as a serial chain of four revolute joints and establish the coordinate frames used throughout the project.

### Variables and Dimensions

- Joint vector: `q in R^4`
- Link parameters: `a_i`, `alpha_i`, `d_i`, `theta_i` for `i = 1, 2, 3, 4`
- Frame set: `{0}, {1}, {2}, {3}, {4}, {tool}`

### Governing Equations

The manipulator configuration is:

```text
q = [q1, q2, q3, q4]^T
```

Each joint is revolute, so the active generalized coordinates are the four joint angles.

### Plain-Language Intuition

The arm is modeled as four rigid links connected in series. Each joint rotates and changes the orientation of all following links. The end-effector pose comes from accumulating these rotations and translations from the base to joint 4 and, in the intended final geometry, onward to the tool frame.

### How the Current Code Realizes It

- `arm4dof_dh.py` defines a `DHLink` dataclass to store the DH parameters of each revolute joint.
- `Arm4DOFDH` stores the four-link chain and exposes the arm degree of freedom through `dof`.
- The code assumes the manipulator is a 4R chain with fixed link geometry and no prismatic joints.

### Assumptions, Simplifications, and Decisions

- All joints are revolute, and the default arm has four links only.
- The project keeps a generic pedagogical 4R geometry so the emphasis remains on modeling and control mathematics rather than hardware-faithful replication.
- The canonical frame structure remains standard DH throughout the baseline and implementation.
- The intended final frame set includes a tool frame after joint 4 so that joint 4 can act as a wrist.
- POE and URDF remain noted as alternative description frameworks for future comparison in the final report.
- End-effector orientation is not yet a first-class controlled variable in the current implementation, but it should become a configurable control objective in later stages, ideally through a code constant or similarly explicit configuration setting.

## Topic 2: Standard DH Convention

### Objective

Formalize how each local frame is related to the next frame using standard Denavit-Hartenberg parameters.

### Variables and Dimensions

- Link length: `a_i`
- Link twist: `alpha_i`
- Link offset: `d_i`
- Joint angle: `theta_i`
- Per-link transform: `T_(i-1)^i in R^(4x4)`

### Governing Equations

The standard DH transform used in the project is:

```text
T_(i-1)^i =
[ cos(theta_i)  -sin(theta_i) cos(alpha_i)   sin(theta_i) sin(alpha_i)   a_i cos(theta_i) ]
[ sin(theta_i)   cos(theta_i) cos(alpha_i)  -cos(theta_i) sin(alpha_i)   a_i sin(theta_i) ]
[      0                sin(alpha_i)               cos(alpha_i)                  d_i        ]
[      0                     0                           0                         1         ]
```

In the current model:

```text
alpha_i = 0
d_i = 0
a_i = [L1, L2, L3, L4]
theta_i = q_i + theta_offset_i
```

A chosen true spatial 4DOF example for later refinement is:

```text
a_i = [0, L2, L3, 0]
alpha_i = [pi/2, 0, 0, 0]
d_i = [L1, 0, 0, 0]
theta_i = q_i + theta_offset_i
```

Any terminal offset should be represented by a separate tool transform `T_4^tool` rather than by `a_4`, so joint 4 behaves as a wrist.

### Plain-Language Intuition

The DH table gives a consistent recipe for moving from one link frame to the next. Each row in the table describes one local rotation and translation. Chaining those rows yields the overall arm pose.

### How the Current Code Realizes It

- `dh_transform(a, alpha, d, theta)` in `arm4dof_dh.py` builds the `4 x 4` homogeneous matrix directly from the standard DH formula.
- `DHLink` stores `a`, `alpha`, `d`, and `theta_offset`.
- The actual joint angle used in the transform is `theta = q + theta_offset`.

### Assumptions, Simplifications, and Decisions

- In the current implementation, every link uses `alpha = 0` and `d = 0`, so there is no twist between successive joint axes and the chain stays planar even though it is rendered in a 3D viewer.
- Standard DH remains the primary convention for the project, while POE and URDF are deferred to future comparison and discussion in the final report.
- A meaningful true spatial 4DOF example should use `a = [0, L2, L3, 0]`, `alpha = [pi/2, 0, 0, 0]`, and `d = [L1, 0, 0, 0]`, together with an explicit tool frame after joint 4. This introduces a base offset and a 90-degree twist at the first joint while keeping joint 4 as a wrist rather than a terminal link extension.
- `theta_offset` represents a fixed constant added to the joint angle before applying the DH transform. In the present code it defaults to zero, so it does not reduce accuracy unless an incorrect offset is entered. For this pedagogical baseline, it should stay zero unless a deliberate home-frame alignment is needed later.
- The final report should include the complete DH table, the frame diagram, and the mathematical explanation for the chosen arm geometry.

## Topic 3: Forward Kinematics

### Objective

Compute the end-effector pose with respect to the base frame from the joint angles.

### Variables and Dimensions

- Input: `q in R^4`
- Current returned pose: `T_0^4(q) in R^(4x4)`
- Planned tool pose: `T_0^tool(q) in R^(4x4)`
- Current end-effector position: `x = T_0^4(q)[0:3, 3] in R^3`
- Planned tool position: `x = T_0^tool(q)[0:3, 3] in R^3`

### Governing Equations

Forward kinematics to the wrist frame is obtained by chaining the four link transforms:

```text
T_0^4(q) = T_0^1(q1) T_1^2(q2) T_2^3(q3) T_3^4(q4)
```

With an explicit tool frame after joint 4:

```text
T_0^tool(q) = T_0^4(q) T_4^tool
```

In compact product notation:

```text
T_0^tool(q) = (prod_(i=1)^4 T_(i-1)^i(q_i)) T_4^tool
```

### Plain-Language Intuition

Forward kinematics answers the question: if the joint angles are known, where is the end-effector and how is it oriented? Instead of solving for the joints from a target point, this step propagates the geometry outward from the base. In the selected final interpretation, joint 4 acts as a wrist and the tool frame sits after it.

### How the Current Code Realizes It

- `Arm4DOFDH.forward_kinematics(q)` starts from the identity matrix.
- It loops through the links, computes each DH transform, and multiplies it into the cumulative transform.
- The returned matrix is currently the frame-4 pose with respect to the base frame.
- Because no explicit tool transform is modeled yet, the current implementation effectively treats frame 4 as the end-effector frame.

### Current

- The current terminal frame is `T_0^4(q)`, not `T_0^tool(q)`.
- The current end-effector motion is confined to the XY plane, with `z = 0` for the default model.
- The rotation submatrix is still computed as a full `3 x 3` orientation matrix, even though the geometry is planar and the net orientation is `R_z(sum_i q_i)`.
- The implemented forward kinematics matches the planar closed form:
  - `x = sum_i L_i cos(sum_(k=1)^i q_k)`
  - `y = sum_i L_i sin(sum_(k=1)^i q_k)`
  - `phi = sum_i q_i`

### Planned

- Later versions should define an explicit tool frame after joint 4 through `T_4^tool`.
- To make joint 4 act as a wrist, `L4` should be set to `0`, and any terminal offset should be represented by `T_4^tool` rather than by a fourth link length.

### Extension

- Forward kinematics should continue returning the full homogeneous transform matrix rather than separately exposing position and orientation-angle outputs.

## Topic 4: Joint-Position Extraction for Visualization

### Objective

Recover the Cartesian coordinates of the base, intermediate joints, and end-effector so the arm can be drawn in the simulation.

### Variables and Dimensions

- Input: `q in R^4`
- Output point set: `p_k in R^3` for `k = 0, 1, 2, 3, 4`
- Plot arrays: `xs`, `ys`, `zs`, each of length `5`

### Governing Equations

If `T_0^k` is the cumulative transform to frame `k`, the origin of that frame is:

```text
p_k = T_0^k[0:3, 3]
```

Collecting all frame origins gives the polyline used to render the arm.

### Plain-Language Intuition

The visualizer does not draw matrices. It draws points. So after computing the cumulative transform of each joint, the code extracts only the translation component and connects those points as the arm skeleton.

### How the Current Code Realizes It

- `Arm4DOFDH.joint_positions(q)` repeats the cumulative transform process used in forward kinematics.
- After each transform multiplication, it stores the current frame origin.
- The function returns separate coordinate arrays for direct plotting in PyVista.

### Assumptions, Simplifications, and Decisions

- The arm is intentionally visualized as a simple polyline rather than cylinders, capsules, or other solid rigid-body renderings.
- Link thickness, joint geometry, and collision shapes are not modeled.
- Center-of-mass positions should not be exposed alongside the joint-origin visualization outputs in this baseline.
- The returned points correspond to frame origins, not necessarily true physical joint housings or centers of mass.
- If a non-identity tool transform is introduced later, the visualizer should treat the tool frame as an additional plotted frame or point rather than folding that offset into `a_4`.
- The plotting helper should continue returning separate coordinate vectors rather than a single `N x 3` point array for now.

## Topic 5: Geometric Jacobian

### Objective

Relate joint velocities to end-effector spatial velocity through a geometric Jacobian.

### Variables and Dimensions

- Input: `q in R^4`
- Output: `J(q) in R^(6x4)`
- Linear velocity block: `J_v in R^(3x4)`
- Angular velocity block: `J_omega in R^(3x4)`

### Governing Equations

The project uses the spatial-velocity form:

```text
xdot = [v; omega] = J(q) q_dot
```

For a revolute joint `i`, the Jacobian column is:

```text
v_i = z_(i-1) x (p_e - p_(i-1))
omega_i = z_(i-1)
```

So the full column is:

```text
J_i = [v_i; omega_i]
```

### Plain-Language Intuition

Each joint contributes to the end-effector velocity in two ways: it can change the end-effector position and it can change the end-effector orientation. For a revolute joint, the linear velocity comes from a cross product between the rotation axis and the lever arm to the end-effector.

### How the Current Code Realizes It

- `jacobian4dof.py` first computes all intermediate transforms `T_0^i`.
- For each joint, it extracts:
  - the axis `z_(i-1)` from the third column of the rotation block,
  - the origin `p_(i-1)` from the translation block.
- It then constructs the Jacobian column from the cross-product rule.

### Current

- The Jacobian is built numerically from the transform chain instead of being derived symbolically.
- In the current implementation, all joint axes remain parallel to the base `z` axis because the arm is still planar.
- The linear block `J_v` has a zero third row in the current geometry, so its rank is at most `2`.
- Away from collinear singularities, the current planar arm achieves `rank(J_v) = 2`; at collinear configurations that rank drops further.
- The full `6 x 4` geometric Jacobian is still useful, but in the current planar model its angular block contributes only base-`z` rotation, so the full Jacobian is generically rank `3`, not `4`.

### Planned

- Once the geometry becomes genuinely spatial, the geometric Jacobian should be interpreted as a true 3D spatial Jacobian, and the linear block `J_v` should be able to reach rank `3` away from singularities.
- The final implementation should continue using only the geometric Jacobian formulation rather than supporting a separate analytic Jacobian.

### Extension

- Deriving a separate planar analytical Jacobian is out of scope for this baseline, but planar closed forms remain useful for validating the current implementation.
- Singularity analysis should be added through an SVD-based treatment, including rank, condition number, and manipulability.

## Topic 6: Damped Least-Squares Inverse Differential Kinematics

### Objective

Map a task-space velocity command to a feasible joint-velocity command while clearly separating the current planar implementation from the planned spatial interface.

### Variables and Dimensions

- Desired task velocity: `v_desired in R^3`
- Embedded spatial command: `xdot_desired in R^6`
- Position Jacobian: `J_v in R^(3x4)`
- Joint velocity output: `q_dot in R^4`
- Damping factor: `lambda > 0`

### Governing Equations

`Current` implementation:

```text
xdot_desired = [v_x, v_y, 0, 0, 0, 0]^T
q_dot = J^+ xdot_desired
```

For the current planar chain, any commanded nonzero `z` linear velocity is unattainable because the third row of `J_v` is identically zero.

`Planned` target:

```text
J_v^+ = J_v^T (J_v J_v^T + lambda^2 I3)^(-1)
q_dot = J_v^+ v_desired
```

### Plain-Language Intuition

The original `6D` spatial-velocity task overconstrains a `4-DOF` manipulator. In the current code, that command is softened by damped least squares and by embedding a planar linear-velocity request into a `6D` vector with zero angular part. Because the current geometry is planar, the actual task that can be tracked is XY position motion plus in-plane orientation change about base `z`. The planned spatial formulation is different: once the arm geometry is upgraded, a `J_v`-based interface can treat `3D` position control as the primary task.

### How the Current Code Realizes It

- `damped_pseudo_inverse(J, lam)` computes the regularized pseudo-inverse.
- `inverse_differential_kinematics(arm, q, xdot_desired, lam)` calls the Jacobian builder and applies the pseudo-inverse.
- When inverse differential kinematics is exercised in the current Cartesian demo, the commanded task is already position-only in practice: the desired linear velocity is embedded into `xdot_desired = [v_desired; 0]`, so the angular part is set to zero.
- The current code still routes that command through the full `6 x 4` Jacobian interface.

### Current

- No joint limits are considered in the inverse differential kinematics step.
- The current demo effectively performs planar XY position tracking, not full spatial `3D` position control.
- The current interface introduces artificial zero-angular-velocity constraints that are not required by the underlying position task.
- No weighted pseudo-inverse or null-space posture control is implemented in the current stage.
- The inverse differential kinematics layer remains at velocity level instead of being embedded in a closed-loop task-space controller with explicit pose error feedback.

### Planned

- The target interface should use the linear Jacobian block `J_v` explicitly so the solver matches the task definition.
- True spatial `3D` position control should be reserved for the future spatial geometry, not claimed for the current planar chain.
- In that future position-only formulation, four joints can satisfy a `3D` position task with up to a one-dimensional null space when `J_v` has full row rank.

### Extension

- The damping parameter should depend on singularity proximity (e.g., via the smallest singular value of `J_v`) rather than remain fixed.
- Position-only `J_v` control, weighted pseudo-inverses, and null-space posture objectives will be discussed as compatible extensions once the spatial geometry is introduced.

## Topic 7: Simplified Joint-Space Dynamics

### Objective

Introduce a minimal dynamic model that relates torque input to joint acceleration.

### Variables and Dimensions

- Joint position: `q in R^4`
- Joint velocity: `q_dot in R^4`
- Joint acceleration: `q_ddot in R^4`
- Torque input: `tau in R^4`
- Inertia matrix: `M(q) in R^(4x4)`
- Damping matrix: `D in R^(4x4)`
- Gravity torque: `g(q) in R^4`

### Governing Equations

The full manipulator dynamics are commonly written as:

```text
M(q) q_ddot + C(q, q_dot) q_dot + g(q) = tau
```

The current project uses the simplified model:

```text
M q_ddot = tau - D q_dot - g(q)
```

with:

```text
M approx diag(I1, I2, I3, I4)
D approx diag(d1, d2, d3, d4)
```

### Plain-Language Intuition

This layer approximates how hard it is to accelerate each joint, how motion is dissipated by damping, and how gravity pulls on the links. It is not a full rigid-body dynamics derivation. It is a teaching model that is simple enough to inspect and simulate while still showing meaningful control effects.

### How the Current Code Realizes It

- `SimpleDynamics4DOF` stores per-joint inertias, dampings, masses, and gravity.
- `mass_matrix()` returns a diagonal matrix from the configured inertias.
- `damping_matrix()` returns a diagonal matrix from the configured dampings.
- `gravity_torque(q)` computes an approximate planar gravity term by summing link contributions.
- `acceleration(q, q_dot, tau)` solves for `q_ddot`.

### Current

- The inertia matrix is diagonal and configuration-independent.
- Coriolis and centrifugal effects are omitted.
- The gravity model is heuristic and based on simplified link center-of-mass reasoning; it is not presented as a physically complete planar gravity derivation.
- Link inertial coupling is ignored.
- The model is suitable for intuition and experimentation, not for high-fidelity physical prediction.

### Planned

- The simplified dynamics model should remain the working model for now; analytically deriving `M(q)`, `C(q, q_dot)`, and `g(q)` is out of scope because the project is not aiming for high-fidelity realism.
- Actuator constraints should be introduced, especially torque limits, torque saturation, and friction.

### Extension

- Lagrangian and recursive Newton-Euler formulations should still be discussed and compared against the simplified model in the final report, but they do not need to be implemented as alternative dynamics layers in the project.

## Topic 8: Joint-Space PID Control

### Objective

Define how joint torques are generated to track desired joint trajectories.

### Variables and Dimensions

- Reference joint position: `q_ref in R^4`
- Reference joint velocity: `q_dot_ref in R^4`
- Position error: `e = q_ref - q`
- Velocity error: `e_dot = q_dot_ref - q_dot`
- Optional integral state: `eta = integral(e dt) in R^4`
- Planned gravity compensation term: `g_hat(q) in R^4`
- Planned commanded torque before saturation: `tau_cmd in R^4`
- Planned applied torque: `tau in R^4`

### Governing Equations

`Current` implementation:

```text
tau_i = Kp_i e_i + Ki_i eta_i + Kd_i e_dot_i
```

In vector form:

```text
tau = Kp e + Ki eta + Kd e_dot
```

`Planned` target:

```text
tau_cmd_i = Kp_i e_i + Ki_i eta_i + Kd_i e_dot_i + g_hat_i(q)
tau_i = sat(tau_cmd_i, tau_min_i, tau_max_i)
```

In vector form:

```text
tau_cmd = Kp e + Ki eta + Kd e_dot + g_hat(q)
tau = sat(tau_cmd)
```

The gain matrices are implemented as independent per-joint scalars. `Ki` may be zero.

### Plain-Language Intuition

In the current code, each joint is driven by an independent PID law. The proportional term pushes the joint toward the reference, the derivative term damps the motion based on velocity mismatch, and the integral term corrects persistent steady-state error when it is used. Gravity compensation, torque saturation, and anti-windup are part of the planned controller refinement, not the current implementation.

### How the Current Code Realizes It

- `PIDGains` stores the scalar gains `kp`, `ki`, and `kd` for each joint.
- `JointSpacePIDController` stores the integral error state.
- `compute_torque(...)` computes the current torque vector from the reference and measured states.
- `simulate_joint_step_response(...)` closes the loop with the dynamics model over a fixed time horizon.
- The current implementation still applies pure joint-space PID; explicit gravity compensation, torque saturation, and anti-windup are not yet active in code.

### Current

- Each joint is controlled independently.
- Integral action remains optional, and the default gains currently used in the demos are effectively PD gains because `Ki = 0`.
- There is no gain scheduling in the baseline controller design.
- The derivative action uses velocity error rather than a filtered finite-difference position error.

### Planned

- The controller refinement should add gravity compensation, torque saturation, and anti-windup without changing the current baseline description of what is already implemented.
- PD plus gravity compensation is the intended default practical mode once that refinement is implemented.

### Extension

- Computed-torque control should be discussed and compared in the formal report only, not implemented as the main controller in this project.

## Topic 9: Numerical Integration

### Objective

Specify how continuous-time equations are stepped forward in time inside the simulations.

### Variables and Dimensions

- Time step: `dt`
- Current state: `(q, q_dot)`
- Next state: `(q_next, q_dot_next)`

### Governing Equations

For the dynamic model, the project uses semi-implicit Euler:

```text
q_ddot = M^(-1) (tau - D q_dot - g(q))
q_dot_next = q_dot + dt q_ddot
q_next = q + dt q_dot_next
```

The current legacy Cartesian demo uses direct velocity integration:

```text
q_next = q + dt q_dot
```

### Plain-Language Intuition

Numerical integration converts the continuous equations into a time-stepped simulation. Semi-implicit Euler is simple, easy to inspect, and often more stable than fully explicit Euler for mechanical systems. The current code uses that integration rule for the dynamic demo, while the legacy Cartesian demo integrates joint velocities directly because it bypasses the dynamics layer.

### How the Current Code Realizes It

- `SimpleDynamics4DOF.step(...)` implements semi-implicit Euler for the dynamic state update.
- `simulation_4dof_3d.py` updates `q` with direct velocity integration in `run_cartesian_demo()`.
- The joint-space step response first simulates the whole trajectory numerically, then replays it in the visualizer.
- The current code does not yet expose a shared integrator abstraction across the project.

### Assumptions, Simplifications, and Decisions

- The time step is fixed.
- No adaptive integration is used.
- The project currently uses two stepping paths: semi-implicit Euler for the dynamic demo and direct `q = q + dt q_dot` integration for the legacy Cartesian demo.
- The project should standardize simulations behind a common integrator interface.
- For this project, that interface should have only one implemented backend: the current semi-implicit Euler approach used by the code.
- Explicit Euler and RK4 should be discussed only in the final report, not added as alternative in-project integrators.
- No numerical error analysis is currently included in the implementation.
- The final report should include a short discussion of numerical stability versus time-step size, including the practical effects of using smaller versus larger `dt`.
- The project does not yet route all simulation stepping through one shared integration abstraction, even though that is now the intended design direction.

## Topic 10: Demo Architecture

The demo layer serves two purposes: it helps debug the implementation and it provides material for reporting. In particular, drawing the end-effector trajectory trace is useful because it makes motion quality and path behavior immediately visible.

The codebase currently contains two implemented demos, while the retained target architecture keeps only one of them as-is and replaces the other with a more complete task-space validation demo.

`Current` implemented demos:

1. Joint-space step response for controller analysis
2. Legacy Cartesian path tracking for differential-kinematics visualization

`Planned` retained architecture:

1. Joint-space step response for controller analysis
2. Task-space sequential target tracking for full-pipeline validation

These roles can be summarized as follows:

The joint-space step response demo isolates the closed-loop dynamics of the system and is used for controller analysis. The current legacy Cartesian demo is a kinematic visualization only. The planned task-space sequential target demo is the version that should eventually exercise the full pipeline, including differential inverse kinematics, control, and dynamics.

### Joint-Space Step Response Demo

#### Objective

Show how the dynamic model and joint-space PID controller move the arm from an initial configuration to a target joint configuration.

#### Variables and Dimensions

- Initial state: `q_init in R^4`, `qdot_init in R^4`
- Reference state: `q_ref in R^4`
- Simulated trajectory: `q(t)`, `q_dot(t)`

#### Governing Equations

The loop combines:

```text
tau = PID(q_ref, q, q_dot)
q_ddot = M^(-1) (tau - D q_dot - g(q))
state update by semi-implicit Euler
```

#### Plain-Language Intuition

This demo answers a control question: if the desired joint angles suddenly change, how does the arm respond over time? It is useful for observing overshoot, damping, settling, and the effect of gain choices.

#### How the Current Code Realizes It

- `simulate_joint_step_response(...)` computes the trajectory offline.
- `run_joint_step_demo()` replays the stored states inside the PyVista scene.
- The arm is drawn as a black polyline and the end-effector as a blue sphere.

#### Assumptions, Simplifications, and Decisions

- The reference is a constant step rather than a time-varying trajectory.
- The same simple dynamics and independent joint PID laws are used throughout.
- The current demo does not yet include explicit gravity compensation, torque saturation, or anti-windup, even though those are part of the intended controller design.
- The step demo should include plots of joint position versus time, joint velocity versus time, error versus time, and torque versus time.
- The minimum reported metrics should include rise time, settling time, and overshoot.
- These plots and metrics are strongly recommended for debugging and reporting, even though they are not yet computed automatically in the current code.
- Gain tuning should remain manual for now rather than being directly connected to `pid_sandbox.py`, but the gains used in each experiment should be documented clearly.
- Multiple reference profiles should be added later rather than now, for example step, ramp, sinusoidal, and different target magnitudes.

### Legacy Cartesian Path-Tracking Demo

#### Objective

Show how differential kinematics can move the current planar end-effector along a target path in the XY plane.

#### Variables and Dimensions

- Cartesian target point: `x_d in R^3`
- Current end-effector point: `x in R^3`
- Desired Cartesian velocity: `v_des in R^3`
- Embedded spatial command: `xdot_desired in R^6`
- Joint velocity command: `q_dot in R^4`

#### Governing Equations

The current demo computes:

```text
v_des = (x_d - x) / dt
xdot_desired = [v_des; 0]
q_dot = J^+ xdot_desired
q_next = q + dt q_dot
```

#### Plain-Language Intuition

This demo visualizes differential kinematics directly. It is useful for seeing how the Jacobian-based velocity map moves the end-effector along a reference path, but it is not a full control-and-dynamics demonstration.

#### How the Current Code Realizes It

- `run_cartesian_demo()` defines a circular path in the XY plane.
- The desired path is shown as a static red point cloud.
- At each step, the current end-effector position is compared with the current target point, a desired velocity is computed, inverse differential kinematics generates `q_dot`, and the joint angles are updated directly with `q = q + dt q_dot`.
- This demo bypasses the joint-space PID controller and the simplified dynamics model.

#### Assumptions, Simplifications, and Decisions

- The desired path is planar and remains confined to the XY plane in the current model.
- End-effector orientation is not actively tracked.
- This is a legacy demo that remains useful for intuition, but it is not the retained long-term full-pipeline validation demo.

### Task-Space Sequential Target Tracking Demo

#### Objective

Exercise the full pipeline with sequential task-space targets so the project can validate overall behavior and realism rather than only isolated controller dynamics.

#### Variables and Dimensions

- Sequential target set: `x_d^(k) in R^3`
- Current end-effector point: `x in R^3`
- Desired Cartesian velocity: `v_des in R^3`
- Joint command or reference update: `q_dot_cmd in R^4`

#### Governing Equations

The intended high-level loop is:

```text
x_d^(k) -> v_des -> inverse differential kinematics -> controller -> dynamics -> q -> x
```

#### Plain-Language Intuition

This demo is the complementary counterpart to the step response demo. Instead of isolating one controller behavior, it checks whether the full kinematics-control-dynamics chain produces believable end-effector motion when the robot is asked to visit a sequence of spatial targets.

#### How the Current Code Realizes It

- This demo is planned as the retained task-space validation demo, but it is not yet implemented in the current baseline code.

#### Assumptions, Simplifications, and Decisions

- This demo should use sequential target points rather than the current legacy Cartesian path-tracking structure.
- All target points should be shown as blue balls, while the currently active target should be highlighted in red.
- The end-effector trajectory trace should be drawn because it is valuable for both debugging and reporting.
- The main focus is full-pipeline behavior and realism, not fine-grained controller tuning in isolation.

## Current Limitations and Upgrade Path

### What Is Already Implemented

- A 4R DH-based manipulator representation.
- Forward kinematics through homogeneous transforms.
- Joint-position extraction for visualization.
- A numerically constructed `6 x 4` geometric Jacobian.
- Damped least-squares inverse differential kinematics.
- A simplified joint-space dynamics layer with diagonal inertia and damping.
- A per-joint PID controller.
- A 3D visualization environment with a joint-space demo and a legacy Cartesian path-tracking demo.

### What Is Not Yet True About the Model

- The arm is not yet a true spatial 4DOF manipulator.
- Although a candidate spatial DH table has now been selected conceptually, the current implementation still does not include nonzero twists or offsets.
- There is not yet an explicit tool transform after joint 4; the current implementation still treats frame 4 as the terminal frame.
- The dynamics are not a full rigid-body derivation.
- Actuator torque limits, torque saturation, and friction are not yet modeled.
- The joint-space controller does not yet include explicit gravity compensation or anti-windup in code.
- The current Cartesian demo bypasses the joint-space PID controller and simplified dynamics, so it is not a full control-and-dynamics demo.
- The inverse differential kinematics layer is not yet a fully closed-loop task-space controller with orientation control.
- There is not yet a systematic SVD-based singularity analysis with rank, condition number, and manipulability, nor is there a workspace or joint-limit analysis in the 4DOF project yet.
- The inverse differential kinematics layer does not yet implement singularity-dependent damping or a cleaned-up `J_v`-based interface in code, even though that is now the intended formulation.
- The simulations are not yet standardized behind a shared integrator abstraction, and the report-level discussion of time-step stability has not yet been written.
- The complementary task-space sequential target demo is part of the intended architecture, but it is not yet implemented in the retained baseline.

### Upgrade Path Toward the Intended Project

The likely progression is:

1. Refine and validate the current mathematical baseline.
2. Keep the arm generic and pedagogical rather than tying it to a specific hardware platform.
3. Introduce the chosen pedagogical spatial DH parameter set with `d1 = L1`, `a1 = 0`, `alpha1 = pi/2`, `a2 = L2`, `a3 = L3`, and `a4 = 0`, together with an explicit tool frame after joint 4.
4. Revisit forward kinematics, Jacobian structure, and path-tracking examples under the new geometry, including optional orientation-control cases.
5. Keep the simplified dynamics model as the main project dynamics layer, but extend it with actuator torque limits, torque saturation, friction, and other practical constraints.
6. Improve control design with gravity compensation, torque saturation, anti-windup, better task-space control, and systematic tuning.
7. Implement the complementary task-space sequential target demo with end-effector trace and reporting outputs.

## Refinement Roadmap

The next iterations should treat each topic below as an independent study and refinement unit:

1. Arm geometry and frame assignment
2. Standard DH table and transform interpretation
3. Forward kinematics derivation and pose interpretation
4. Joint-position extraction and visualization model
5. Jacobian derivation, meaning, and singularities
6. Inverse differential kinematics and damping strategy
7. Dynamic model validity and alternatives
8. Joint-space PID design and tuning logic
9. Numerical integration choices and simulation stability
10. Demo architecture, validation plots, and interpretation of results
11. Gap analysis between the current planar-in-3D model and a true spatial arm
12. Structure for the final academic report and simplified README

## Baseline Conclusion

The current 4DOF project already contains a coherent educational pipeline:

- define a DH-based manipulator,
- compute forward kinematics,
- derive and use the Jacobian,
- add a simple dynamics model,
- apply joint-space control,
- visualize the results in 3D.

The main conceptual caution is that the present implementation is a planar 4R model rendered in a 3D environment, not yet a fully spatial manipulator. This document should therefore be treated as the authoritative baseline for understanding what exists now, what assumptions it uses, and what must change in order to evolve the project into a more realistic 4DOF 3D robotic arm study.



## More topics to discuss in formal report
- Analytical vs geometric Jacobian
- DLS pseudo-inverse vs standard pseudo-inverse
- Position only Jacobian and what it means for a 4DOF arm
- How inverse kinematics with full Jacobian could be resolved with aproximations using DLS and task priorization (weights)
- Kinematics layer vs control layer
- Feedback to kinematics and control layer
- Simple dynamics vs Lagrangian vs Newton-Euler
- Forward dynamics vs Inverse dynamics
- PID + gravity compensation vs Computed torque (inverse dynamics)
- Gain tuning
- Semi-implicit Euler vs Explicit Euler vs RK4
- Differential IK vs Numerical Integration (similar but distinct things)
- Explain all the different layers and their inputs/outputs (Measure -> Compute Error -> Differential IK -> Controller -> Dynamics -> Numerical Integration)
- Workspace analysis
