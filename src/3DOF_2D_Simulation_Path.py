import matplotlib

matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple, Optional


class PlanarArm3DOF:
    """
    Encapsulates a planar 3R manipulator simulation.
    Handles kinematics, control, trajectory generation, and visualization.
    """

    def __init__(
            self,
            l1: float = 10.0,
            l2: float = 5.0,
            l3: float = 3.0,
            kp: float = 0.05,
            dt: float = 0.02
    ) -> None:
        """
        Initialize model parameters and visualization.
        Fully prepares the simulation environment upon instantiation.
        """
        self.L1: float = l1
        self.L2: float = l2
        self.L3: float = l3

        self.KP: float = kp
        self.DT: float = dt

        self.theta1: float = 0.0
        self.theta2: float = 0.0
        self.theta3: float = 0.0

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-20.0, 20.0)
        self.ax.set_ylim(-20.0, 20.0)
        self.ax.set_title("3DOF Arm Simulation")
        self.ax.grid(False)

        x, y = self.forward_kinematics(
            self.theta1, self.theta2, self.theta3
        )

        self.arm_line, = self.ax.plot(
            x, y, 'o-', color='black', linewidth=3
        )
        self.joints, = self.ax.plot(
            x, y, 'o', color='blue', markersize=5
        )

        self.orientation_arrow = self.ax.arrow(0, 0, 0, 0)
        self.orientation_arrow = self.draw_arrow(
            self.theta1, self.theta2, self.theta3
        )

    def forward_kinematics(
            self,
            t1: float,
            t2: float,
            t3: float
    ) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
        """
        Compute planar forward kinematics of the 3R manipulator.
        Returns x and y coordinates of base, joints, and end-effector.
        """
        x1: float = self.L1 * np.cos(t1)
        y1: float = self.L1 * np.sin(t1)

        x2: float = x1 + self.L2 * np.cos(t1 + t2)
        y2: float = y1 + self.L2 * np.sin(t1 + t2)

        x3: float = x2 + self.L3 * np.cos(t1 + t2 + t3)
        y3: float = y2 + self.L3 * np.sin(t1 + t2 + t3)

        return (0.0, x1, x2, x3), (0.0, y1, y2, y3)

    def inverse_kinematics(
            self,
            x_target: float,
            y_target: float,
            phi_target: float = 0.0,
            elbow_up: bool = True
    ) -> Optional[Tuple[float, float, float]]:
        """
        Analytical inverse kinematics for a planar 3R arm.
        Computes joint angles for desired position and orientation.
        Returns None if the target is unreachable.
        """
        x_w: float = x_target - self.L3 * np.cos(phi_target)
        y_w: float = y_target - self.L3 * np.sin(phi_target)

        r2: float = x_w ** 2 + y_w ** 2
        cos_theta2: float = (
                (r2 - self.L1 ** 2 - self.L2 ** 2)
                / (2 * self.L1 * self.L2)
        )

        if abs(cos_theta2) > 1.0:
            return None

        sin_theta2: float = np.sqrt(1 - cos_theta2 ** 2)
        if not elbow_up:
            sin_theta2 = -sin_theta2

        theta2: float = np.arctan2(sin_theta2, cos_theta2)

        theta1: float = (
                np.arctan2(y_w, x_w)
                - np.arctan2(self.L2 * sin_theta2,
                             self.L1 + self.L2 * cos_theta2)
        )

        theta3: float = phi_target - theta1 - theta2

        return theta1, theta2, theta3

    @staticmethod
    def shortest_angle_diff(target: float, current: float) -> float:
        """
        Compute minimal signed angular difference.
        Result is wrapped to the interval [-pi, pi].
        """
        return (target - current + np.pi) % (2 * np.pi) - np.pi

    def generate_path(
            self,
            pattern: str = "random",
            num_points: int = 20
    ) -> List[Tuple[float, float]]:
        """
        Generate a Cartesian trajectory inside the reachable workspace.
        Supports 'random' and 'spiral' patterns.
        """
        radius: float = self.L1 + self.L2 + self.L3
        points: List[Tuple[float, float]] = []

        if pattern == "random":
            for _ in range(num_points):
                r: float = np.random.uniform(
                    0.5 * (self.L1 + self.L2),
                    0.95 * radius
                )
                theta: float = np.random.uniform(0.0, 2.0 * np.pi)
                points.append((r * np.cos(theta), r * np.sin(theta)))

        elif pattern == "spiral":
            thetas = np.linspace(0.0, 4.0 * np.pi, num_points)
            rs = np.linspace(
                0.5 * (self.L1 + self.L2),
                0.9 * radius,
                num_points
            )
            for r, t in zip(rs, thetas):
                points.append((r * np.cos(t), r * np.sin(t)))

        return points

    def proportional_update_arm(
            self,
            angles: Tuple[float, float, float]
    ) -> None:
        """
        Apply proportional control in joint space.
        Updates joint states toward target angles.
        """
        t1_target, t2_target, t3_target = angles

        self.theta1 += self.KP * self.shortest_angle_diff(
            t1_target, self.theta1
        )
        self.theta2 += self.KP * self.shortest_angle_diff(
            t2_target, self.theta2
        )
        self.theta3 += self.KP * self.shortest_angle_diff(
            t3_target, self.theta3
        )

        self.update_arm(self.theta1, self.theta2, self.theta3)

    def update_arm(self, t1: float, t2: float, t3: float) -> None:
        """
        Update graphical representation of the manipulator.
        Redraws links, joints, and orientation arrow.
        """
        x_values, y_values = self.forward_kinematics(t1, t2, t3)

        self.arm_line.set_data(x_values, y_values)
        self.joints.set_data(x_values, y_values)

        self.draw_arrow(t1, t2, t3)
        self.fig.canvas.draw_idle()

    def draw_arrow(self, t1: float, t2: float, t3: float):
        """
        Draw end-effector orientation arrow.
        Arrow direction equals total joint angle.
        """
        x_values, y_values = self.forward_kinematics(t1, t2, t3)
        phi: float = t1 + t2 + t3

        self.orientation_arrow.remove()
        self.orientation_arrow = self.ax.arrow(
            x_values[-1],
            y_values[-1],
            np.cos(phi),
            np.sin(phi),
            head_width=0.6,
            head_length=0.8,
            fc='black',
            ec='black'
        )

        return self.orientation_arrow

    def path_mode(
            self,
            pattern: str = "random",
            num_points: int = 20
    ) -> None:
        """
        Execute automatic trajectory tracking.
        Uses inverse kinematics and proportional control.
        """
        points = self.generate_path(pattern, num_points)

        for px, py in points:
            self.ax.plot(px, py, 'o', color='red', markersize=4)

        path_idx: List[int] = [0]
        tolerance: float = 0.15

        def timer_event(_) -> None:
            point_x, point_y = points[path_idx[0]]
            phi_target: float = np.arctan2(point_y, point_x)

            angles = self.inverse_kinematics(
                point_x, point_y, phi_target
            )

            if angles is None:
                path_idx[0] = (path_idx[0] + 1) % len(points)
                return

            self.proportional_update_arm(angles)

            x_vals, y_vals = self.forward_kinematics(
                self.theta1, self.theta2, self.theta3
            )

            if np.hypot(
                    point_x - x_vals[-1],
                    point_y - y_vals[-1]
            ) < tolerance:
                path_idx[0] = (path_idx[0] + 1) % len(points)

        timer = self.fig.canvas.new_timer(
            interval=int(self.DT * 1000)
        )
        timer.add_callback(timer_event, None)
        timer.start()
        plt.show()


def main() -> None:
    """
    Entry point of the simulation.
    Instantiates the arm and starts trajectory execution.
    """
    arm = PlanarArm3DOF()
    arm.path_mode(pattern="random", num_points=20)


if __name__ == "__main__":
    main()
