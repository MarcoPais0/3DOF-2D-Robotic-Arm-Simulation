import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

L1, L2, L3 = 10, 5, 3
fig, ax = plt.subplots()


def workspace_analysis():
    """
    Draw workspace as concentric rings.
    For each radius (100 steps), rotate 200 times.
    """
    r_max = L1 + L2 + L3
    r_min = max(0, L1 - L2 - L3)
    radius_steps = 100
    angle_steps = 100

    radii = np.linspace(r_min, r_max, radius_steps)
    for r in radii:
        for theta in np.linspace(0, 2*np.pi, angle_steps):

            x = r * np.cos(theta)
            y = r * np.sin(theta)
            ax.plot(x, y, '.', color='red', markersize=1)

        fig.canvas.draw_idle()
        plt.pause(0.1)
        angle_steps += 1

    plt.show()


def setup():
    """Set up the simulation environment."""
    global ax
    ax.set_aspect('equal')
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_title("3DOF Arm Simulation Workspace")
    ax.grid(True)


def main():
    setup()
    workspace_analysis()


if __name__ == "__main__":
    main()
