import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backend_bases import MouseEvent

# ------------------- Arm Parameters -------------------
L1, L2, L3 = 10, 5, 3
THETA1, THETA2, THETA3 = 0.0, 0.0, 0.0
Kp = 0.05  # proportional gain for smooth movement
dt = 0.02  # timer interval in seconds


# ------------------- Kinematics -------------------
def forward_kinematics(t1, t2, t3):
    """Compute the joint positions of the 3DOF planar arm."""
    x1 = L1 * np.cos(t1)
    y1 = L1 * np.sin(t1)
    x2 = x1 + L2 * np.cos(t1 + t2)
    y2 = y1 + L2 * np.sin(t1 + t2)
    x3 = x2 + L3 * np.cos(t1 + t2 + t3)
    y3 = y2 + L3 * np.sin(t1 + t2 + t3)
    return (0, x1, x2, x3), (0, y1, y2, y3)


def inverse_kinematics(x_target, y_target, phi_target=0.0, elbow_up=True):
    """
    Analytical inverse kinematics for a 3R planar manipulator.

    Parameters:
        x_target: desired end-effector position
        y_target : desired end-effector position
        phi_target         : desired end-effector orientation (radians)
        elbow_up           : True for elbow-up, False for elbow-down

    Returns:
        (theta1, theta2, theta3) or None if unreachable
    """

    # -------- Step 1: Compute Wrist Position --------
    x_w = x_target - L3 * np.cos(phi_target)
    y_w = y_target - L3 * np.sin(phi_target)

    # -------- Step 2: Compute r^2 --------
    r2 = x_w ** 2 + y_w ** 2

    # -------- Step 3: Law of Cosines for theta2 --------
    cos_theta2 = (r2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)

    # Workspace check
    if abs(cos_theta2) > 1.0:
        return None

    sin_theta2 = np.sqrt(1 - cos_theta2 ** 2)

    # Elbow configuration selection
    if not elbow_up:
        sin_theta2 = -sin_theta2

    theta2 = np.arctan2(sin_theta2, cos_theta2)

    # -------- Step 4: Compute theta1 --------
    theta1 = (
            np.arctan2(y_w, x_w)
            - np.arctan2(L2 * sin_theta2, L1 + L2 * cos_theta2)
    )

    # -------- Step 5: Compute theta3 --------
    theta3 = phi_target - theta1 - theta2

    return theta1, theta2, theta3


# ------------------- Utilities -------------------
def shortest_angle_diff(target, current):
    """Compute minimal signed angular difference (-pi to pi)."""
    return (target - current + np.pi) % (2 * np.pi) - np.pi


def generate_path(pattern="circle", num_points=20):
    """Generate a list of (x, y) points for the arm to follow."""
    radius = L1 + L2 + L3
    points = []

    if pattern == "circle":
        angles = np.linspace(0, 2 * np.pi, num_points)
        for a in angles:
            points.append((radius * 0.8 * np.cos(a), radius * 0.8 * np.sin(a)))

    elif pattern == "random":
        for _ in range(num_points):
            r = np.random.uniform(0.5 * (L1 + L2), 0.95 * radius)
            theta = np.random.uniform(0, 2 * np.pi)
            points.append((r * np.cos(theta), r * np.sin(theta)))

    elif pattern == "spiral":
        thetas = np.linspace(0, 4 * np.pi, num_points)
        rs = np.linspace(0.5 * (L1 + L2), 0.9 * radius, num_points)
        for r, t in zip(rs, thetas):
            points.append((r * np.cos(t), r * np.sin(t)))

    return points


# ------------------- Plot Setup -------------------
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_title("3DOF Arm Simulation")
ax.grid(True)

x_vals, y_vals = forward_kinematics(THETA1, THETA2, THETA3)
arm_line, = ax.plot(x_vals, y_vals, 'o-', color='black', linewidth=3)
joints, = ax.plot(x_vals, y_vals, 'o', color='green', markersize=8)
target_dot, = ax.plot([x_vals[-1]], [y_vals[-1]], 'ro', markersize=6)

# Global target position (for click or path)
target_pos = [x_vals[-1], y_vals[-1]]


# ------------------- Arm Update -------------------
def update_arm(t1, t2, t3):
    """Update the arm and target dot positions on the plot."""
    x_values, y_values = forward_kinematics(t1, t2, t3)
    arm_line.set_data(x_values, y_values)
    joints.set_data(x_values, y_values)
    target_dot.set_data([target_pos[0]], [target_pos[1]])
    fig.canvas.draw_idle()


def smooth_update_arm(angles):
    global THETA1, THETA2, THETA3
    t1_target, t2_target, t3_target = angles
    THETA1 += Kp * shortest_angle_diff(t1_target, THETA1)
    THETA2 += Kp * shortest_angle_diff(t2_target, THETA2)
    THETA3 += Kp * shortest_angle_diff(t3_target, THETA3)
    update_arm(THETA1, THETA2, THETA3)


# ------------------- Modes -------------------
def path_mode(pattern="circle"):
    """Automatically follow a predefined path."""
    points = generate_path(pattern)
    for px, py in points:
        ax.plot(px, py, 'ro', markersize=5)

    global target_pos
    target_pos = [points[0][0], points[0][1]]
    path_idx = [0]
    tolerance = 0.1

    def timer_event(_):
        nonlocal points, path_idx
        point_x, point_y = points[path_idx[0]]
        phi_target = np.arctan2(point_x, point_y)
        angles = inverse_kinematics(point_x, point_y, phi_target, elbow_up=True)
        if angles:
            smooth_update_arm(angles)

            # Switch to next point once reached
            x_ee, y_ee = forward_kinematics(THETA1, THETA2, THETA3)[0][-1], \
                forward_kinematics(THETA1, THETA2, THETA3)[1][-1]
            if np.hypot(point_x - x_ee, point_y - y_ee) < tolerance:
                path_idx[0] = (path_idx[0] + 1) % len(points)

    timer = fig.canvas.new_timer(interval=int(dt * 1000))
    timer.add_callback(timer_event, None)
    timer.start()
    plt.show()


def click_mode():
    """Move arm to clicked position, showing a red target dot."""
    global target_pos
    target_pos = [x_vals[-1], y_vals[-1]]

    def onclick(event: MouseEvent) -> None:
        if event.xdata is not None and event.ydata is not None:
            target_pos[0] = event.xdata
            target_pos[1] = event.ydata

    fig.canvas.mpl_connect('button_press_event', onclick)

    def timer_event(_):
        phi_target = np.arctan2(target_pos[0], target_pos[1])
        angles = inverse_kinematics(target_pos[0], target_pos[1], phi_target, elbow_up=True)
        if angles:
            smooth_update_arm(angles)

    timer = fig.canvas.new_timer(interval=int(dt * 1000))
    timer.add_callback(timer_event, None)
    timer.start()
    plt.show()


# ------------------- Main -------------------
def main():
    # Switch mode here:
    path_mode(pattern="random")  # Options: "circle", "random", "spiral"
    # click_mode()


if __name__ == "__main__":
    main()
