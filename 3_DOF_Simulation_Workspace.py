import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np


class WorkspaceAnalyzer:
    def __init__(self, l1: float = 10, l2: float = 5, l3: float = 3) -> None:
        """
        Initialize the workspace analyzer with robot link lengths and figure setup.
        """
        self.L1: float = l1
        self.L2: float = l2
        self.L3: float = l3

        # Figure and axis setup
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        self.ax.set_title("3DOF Arm Simulation Workspace")
        self.ax.grid(False)

    def workspace_analysis(self) -> None:
        """
        Draw workspace as concentric rings.
        For each radius, rotate the end-effector 200 times.
        """
        r_max: float = self.L1 + self.L2 + self.L3
        r_min: float = max(0.0, self.L1 - self.L2 - self.L3)
        radius_steps: int = 100
        angle_steps: int = 100

        radii: np.ndarray = np.linspace(r_min, r_max, radius_steps)
        for r in radii:
            angles: np.ndarray = np.linspace(0, 2 * np.pi, angle_steps)
            x: np.ndarray = r * np.cos(angles)
            y: np.ndarray = r * np.sin(angles)

            self.ax.plot(x, y, '.', color='red', markersize=1)
            self.fig.canvas.draw_idle()
            angle_steps += 1
            plt.pause(0.05)

        plt.show()


def main() -> None:
    analyzer = WorkspaceAnalyzer()
    analyzer.workspace_analysis()


if __name__ == "__main__":
    main()
