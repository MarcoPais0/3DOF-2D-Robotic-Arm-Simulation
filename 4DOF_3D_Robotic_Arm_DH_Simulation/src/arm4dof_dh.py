import numpy as np
from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass
class DHLink:
    """
    Standard Denavit–Hartenberg (DH) parameters for a single revolute joint.

    All joints are assumed revolute. The parameters (a, alpha, d, theta_offset)
    are defined using the *standard* DH convention. The actual joint angle is
    theta = q + theta_offset.
    """

    a: float
    alpha: float
    d: float
    theta_offset: float = 0.0


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """
    Compute the standard DH homogeneous transform for a single link.

    Parameters
    ----------
    a : float
        Link length (distance along x_{i} from z_{i} to z_{i+1}).
    alpha : float
        Link twist (angle between z_{i} and z_{i+1} about x_{i}).
    d : float
        Link offset (distance along z_{i} from x_{i-1} to x_{i}).
    theta : float
        Joint angle (rotation about z_{i}).

    Returns
    -------
    T : (4, 4) ndarray
        Homogeneous transformation matrix from frame i to i+1.
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


class Arm4DOFDH:
    """
    Simple 4DOF revolute manipulator defined via standard DH parameters.

    This model is intentionally kept pedagogical:
    - 4 revolute joints (R-R-R-R).
    - All joint axes are parallel (planar kinematics), but the arm is
      embedded and visualised in 3D.
    - Link lengths are configurable.

    The arm configuration is:
        q = [q1, q2, q3, q4]^T

    The default DH table (standard convention) uses:
        alpha_i = 0
        d_i     = 0
        a_i     = [L1, L2, L3, L4]
    """

    def __init__(
        self,
        l1: float = 4.0,
        l2: float = 3.0,
        l3: float = 2.0,
        l4: float = 1.0,
    ) -> None:
        self.links = [
            DHLink(a=l1, alpha=0.0, d=0.0),
            DHLink(a=l2, alpha=0.0, d=0.0),
            DHLink(a=l3, alpha=0.0, d=0.0),
            DHLink(a=l4, alpha=0.0, d=0.0),
        ]

    @property
    def dof(self) -> int:
        return len(self.links)

    def forward_kinematics(self, q: Sequence[float]) -> np.ndarray:
        """
        Compute the homogeneous transform from base to end-effector.

        Parameters
        ----------
        q : array_like, shape (4,)
            Joint angles [q1, q2, q3, q4].

        Returns
        -------
        T_0_4 : (4, 4) ndarray
            Homogeneous transform of the end-effector frame with respect to
            the base frame.
        """
        if len(q) != self.dof:
            raise ValueError(f"Expected {self.dof} joint angles, got {len(q)}.")

        T = np.eye(4)
        for qi, link in zip(q, self.links):
            theta = qi + link.theta_offset
            T = T @ dh_transform(link.a, link.alpha, link.d, theta)
        return T

    def joint_positions(self, q: Sequence[float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute 3D positions of all joint frames (including base and end-effector).

        Parameters
        ----------
        q : array_like, shape (4,)
            Joint angles.

        Returns
        -------
        xs, ys, zs : ndarray
            Arrays of x, y, z coordinates of the base, intermediate joints,
            and end-effector, each of length 5.
        """
        if len(q) != self.dof:
            raise ValueError(f"Expected {self.dof} joint angles, got {len(q)}.")

        T = np.eye(4)
        points = [T[:3, 3].copy()]
        for qi, link in zip(q, self.links):
            theta = qi + link.theta_offset
            T = T @ dh_transform(link.a, link.alpha, link.d, theta)
            points.append(T[:3, 3].copy())

        pts = np.stack(points, axis=0)
        return pts[:, 0], pts[:, 1], pts[:, 2]


def main() -> None:
    """
    Small sanity check: print the end-effector transform for a sample q.
    """
    arm = Arm4DOFDH()
    q = np.deg2rad([30.0, 20.0, -15.0, 40.0])
    T = arm.forward_kinematics(q)
    np.set_printoptions(precision=3, suppress=True)
    print("T_0_4(q):")
    print(T)


if __name__ == "__main__":
    main()

