# 4DOF 3D Robotic Arm Simulation (DH, Differential Kinematics, and Simple Dynamics)

## Overview

This project extends the previous 3DOF planar arm simulation to a 4DOF arm
modelled using the **standard Denavit–Hartenberg (DH)** convention. It adds:

- A 4R manipulator defined via DH parameters.
- Forward kinematics and 3D visualization.
- A numerically constructed **geometric Jacobian** and **inverse differential
  kinematics**.
- A simplified joint-space **dynamics layer** with inertia, damping, and
  approximate gravity torques.
- Per-joint **PD/PID controllers** for joint-space tracking.
- Example simulations:
  - Joint-space step responses.
  - A legacy Cartesian trajectory-tracking demo via differential kinematics.

The emphasis is on clarity and educational value rather than physical realism.

## Kinematic Modelling with Standard DH

The arm is represented as a chain of four revolute joints:
\( q = [q_1, q_2, q_3, q_4]^T \).

Using the standard DH convention, each link is characterised by the parameters
\( (a_i, \alpha_i, d_i, \theta_i) \). For this project:

- All joints are revolute.
- We choose a simple planar chain with:
  - \( \alpha_i = 0 \)
  - \( d_i = 0 \)
  - \( a_i = [L_1, L_2, L_3, L_4] \)

This means the current arm is a **planar 4R chain embedded in a 3D viewer**:
the end-effector moves in the XY plane, the Z coordinate remains zero, and the
net orientation is a rotation about the base Z axis.

The per-link homogeneous transform is:
\[
T_i^{i+1} =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i  & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0            & \sin\alpha_i               & \cos\alpha_i              & d_i \\
0            & 0                          & 0                         & 1
\end{bmatrix}
\]

Forward kinematics is obtained by chaining the transforms:
\[
T_0^4(q) = \prod_{i=1}^{4} T_{i-1}^{i}(q_i)
\]

Implementation:

- `arm4dof_dh.py`:
  - `DHLink` dataclass for DH parameters.
  - `dh_transform(a, alpha, d, theta)` builds the homogeneous transform.
  - `Arm4DOFDH` encapsulates the arm and provides:
    - `forward_kinematics(q) -> T_0_4`
    - `joint_positions(q) -> (xs, ys, zs)` for visualization.

## Differential Kinematics and Jacobian

The **geometric Jacobian** relates joint velocities to end-effector spatial
velocity:
\[
\dot{x} =
\begin{bmatrix}
v \\
\omega
\end{bmatrix}
 = J(q) \dot{q}
\]

`jacobian4dof.py` constructs \( J(q) \) numerically:

- First, compute all intermediate transforms \( T_0^i \) along the chain.
- For each joint \( i \):
  - Extract the joint axis \( z_{i-1} \) and origin \( p_{i-1} \).
  - Compute:
    - \( v_i = z_{i-1} \times (p_e - p_{i-1}) \)
    - \( \omega_i = z_{i-1} \)
- Stack the columns to form a \( 6 \times 4 \) Jacobian.

For the current planar geometry, this Jacobian has an important structural
limitation:

- The linear block \( J_v(q) \) has a zero third row, so its rank is at most 2.
- Away from collinear singularities, the current arm therefore provides planar
  XY position motion plus orientation change about the base Z axis, not full
  spatial 3D position reachability.

For **inverse differential kinematics**, a damped least-squares
pseudo-inverse is used:
\[
J^+ = (J^T J + \lambda^2 I)^{-1} J^T
\]
and:
\[
\dot{q} = J^+ \dot{x}_{\text{desired}}
\]

In the current Cartesian demo, the desired linear velocity is embedded into a
\( 6D \) spatial command with zero angular part, and the full geometric
Jacobian is used. This improves robustness near singular configurations at the
cost of small tracking errors, but it should not be interpreted as full spatial
\( 3D \) position control for the present planar arm.

- `geometric_jacobian(arm, q) -> J`
- `inverse_differential_kinematics(arm, q, xdot_desired, lam)`

## Simplified Dynamics

The true dynamics of a manipulator can be written as:
\[
M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + g(q) = \tau
\]

Here, we use a simplified model implemented in `dynamics4dof.py`:

- Approximate the mass matrix as diagonal:
  - \( M(q) \approx \text{diag}(I_1, I_2, I_3, I_4) \)
- Approximate damping as:
  - \( D = \text{diag}(d_1, d_2, d_3, d_4) \)
- Provide a crude gravity-torque approximation based on:
  - Link lengths \( a_i \)
  - Link masses \( m_i \)
  - CoM at half the link length.

Joint accelerations are obtained from:
\[
M \ddot{q} = \tau - D \dot{q} - g(q)
\]
and integrated forward using semi-implicit Euler.

Main components:

- `SimpleDynamics4DOF`:
  - Stores per-joint inertias, dampings, and link masses.
  - `acceleration(q, q_dot, tau)` returns \( \ddot{q} \).
  - `step(q, q_dot, tau, dt)` advances the state.

This is sufficient to study the influence of inertia, damping, and gravity on
joint trajectories under control.

## Joint-Space PD/PID Control

`control4dof.py` implements a per-joint **PID controller**:

\[
\tau_i = K_{p,i} e_i + K_{i,i} \int e_i \, dt + K_{d,i} \dot{e}_i
\]

with:

- \( e_i = q_{i,\text{ref}} - q_i \)
- \( \dot{e}_i = \dot{q}_{i,\text{ref}} - \dot{q}_i \)

Key elements:

- `PIDGains(kp, ki, kd)` for each joint.
- `JointSpacePIDController`:
  - Maintains integral errors.
  - `compute_torque(q, q_dot, q_ref, q_dot_ref, dt)` outputs joint torques.

There is also a helper function:

- `simulate_joint_step_response(...)`:
  - Runs a joint-space step response simulation using the dynamics and PID
    controller.
  - Returns time histories of \( q(t) \) and \( \dot{q}(t) \).

This lets you experiment with different gain settings and observe overshoot,
settling time, and steady-state error.

## 3D Simulation and Demos (PyVista)

`simulation_4dof_3d.py` provides visualization and demos using **PyVista** for
interactive 3D rendering.

Two main demos are included:

1. **Joint-space step response** (`run_joint_step_demo`):
   - Uses `simulate_joint_step_response` to generate trajectories.
   - Animates the arm in 3D as all joints move from zero to a target
     configuration under PID control, using a PyVista `Plotter` with the arm
     rendered as a polyline and the end-effector as a small sphere.

2. **Cartesian trajectory tracking** (`run_cartesian_demo`):
   - Defines a circular path for the end-effector in the XY plane.
   - At each time step:
     - Compute the desired Cartesian velocity toward the next point.
     - Use `inverse_differential_kinematics` to obtain joint velocities.
     - Integrate the joint angles directly with \( q_{k+1} = q_k + dt \, \dot{q}_k \).
   - Animates the arm and shows the desired Cartesian path as a static red
     point cloud.

   This second demo is a **legacy differential-kinematics visualization**. It
   does **not** use the joint-space PID controller or the simplified dynamics
   model end-to-end.

The visualization pipeline is:

- Use `Arm4DOFDH.joint_positions(q)` to obtain the 3D joint coordinates.
- Build a line mesh from these points via `pyvista.lines_from_points`.
- Update the mesh points and an end-effector sphere in each simulation step,
  followed by `plotter.render()` to refresh the view.

The joint-space demo combines DH-based kinematics, simple dynamics, control,
and PyVista rendering. The Cartesian demo is a Jacobian-based kinematic
visualization that is currently separate from the dynamics-and-control loop.

## Possible Extensions

- Refine the arm geometry and DH parameters to match an actual industrial
  manipulator.
- Improve the dynamics by deriving an analytical \( M(q) \), \( C(q, \dot{q}) \),
  and \( g(q) \).
- Add **gravity compensation** to the controller:
  \[
  \tau = \tau_{\text{PID}} + g(q)
  \]
- Implement Ziegler–Nichols or other automatic tuning rules for one joint and
  compare tuned vs manually selected gains.
- Extend the Cartesian controller by closing a loop in task space (e.g., using
  a resolved-rate motion control scheme).
