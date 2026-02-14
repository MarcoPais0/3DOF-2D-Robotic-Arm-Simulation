import numpy as np
from typing import Sequence

try:
    # When used as part of the package
    from .arm4dof_dh import Arm4DOFDH, dh_transform
except ImportError:
    # When run directly from the src directory
    from arm4dof_dh import Arm4DOFDH, dh_transform


def geometric_jacobian(arm: Arm4DOFDH, q: Sequence[float]) -> np.ndarray:
    """
    Compute the 6x4 geometric Jacobian for the 4DOF DH arm.

    The Jacobian is constructed numerically from the DH chain using the
    classic z-axis / cross-product method:
        v_i   = z_i-1 x (p_e - p_i-1)
        w_i   = z_i-1

    Parameters
    ----------
    arm : Arm4DOFDH
        Arm model providing DH parameters.
    q : array_like, shape (4,)
        Joint angles.

    Returns
    -------
    J : (6, 4) ndarray
        Geometric Jacobian mapping joint velocities to end-effector spatial
        velocity [v; w].
    """
    if len(q) != arm.dof:
        raise ValueError(f"Expected {arm.dof} joint angles, got {len(q)}.")

    Ts = []
    T = np.eye(4)
    Ts.append(T.copy())
    for qi, link in zip(q, arm.links):
        theta = qi + link.theta_offset
        T = T @ dh_transform(link.a, link.alpha, link.d, theta)
        Ts.append(T.copy())

    p_e = Ts[-1][:3, 3]
    J = np.zeros((6, arm.dof), dtype=float)

    for i in range(arm.dof):
        z_i = Ts[i][:3, 2]
        p_i = Ts[i][:3, 3]
        v_i = np.cross(z_i, p_e - p_i)
        w_i = z_i
        J[0:3, i] = v_i
        J[3:6, i] = w_i

    return J


def damped_pseudo_inverse(J: np.ndarray, lam: float = 1e-3) -> np.ndarray:
    """
    Compute the damped least-squares pseudo-inverse of a Jacobian.

    Parameters
    ----------
    J : ndarray, shape (m, n)
        Jacobian matrix.
    lam : float
        Damping factor (lambda). Larger values give more robustness near
        singularities but less accuracy.

    Returns
    -------
    J_pinv : ndarray, shape (n, m)
        Damped pseudo-inverse of J.
    """
    m, n = J.shape
    if m >= n:
        return np.linalg.inv(J.T @ J + (lam ** 2) * np.eye(n)) @ J.T
    return J.T @ np.linalg.inv(J @ J.T + (lam ** 2) * np.eye(m))


def inverse_differential_kinematics(
    arm: Arm4DOFDH,
    q: Sequence[float],
    xdot_desired: np.ndarray,
    lam: float = 1e-3,
) -> np.ndarray:
    """
    Compute joint velocities from a desired end-effector spatial velocity.

    Parameters
    ----------
    arm : Arm4DOFDH
        Arm model.
    q : array_like, shape (4,)
        Current joint configuration.
    xdot_desired : ndarray, shape (6,)
        Desired spatial velocity [v; w] of the end-effector.
    lam : float
        Damping factor for the pseudo-inverse.

    Returns
    -------
    qdot : ndarray, shape (4,)
        Joint velocities realizing the desired spatial velocity in the
        least-squares sense.
    """
    J = geometric_jacobian(arm, q)
    J_pinv = damped_pseudo_inverse(J, lam=lam)
    return J_pinv @ xdot_desired


def main() -> None:
    """
    Small demo: print the Jacobian at a sample configuration.
    """
    arm = Arm4DOFDH()
    q = np.deg2rad([30.0, 20.0, -15.0, 40.0])
    J = geometric_jacobian(arm, q)
    np.set_printoptions(precision=3, suppress=True)
    print("J(q):")
    print(J)


if __name__ == "__main__":
    main()

