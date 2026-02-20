import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

L1, L2, L3 = 10, 5, 3
THETA1, THETA2, THETA3 = 0.0, 0.0, 0.0
Kp = 0.05
dt = 0.02

fig, ax = plt.subplots()
arm_line, = ax.plot([], [])
joints, = ax.plot([], [])

orientation_arrow = ax.arrow(0, 0, 0, 0)


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

    x_w = x_target - L3 * np.cos(phi_target)
    y_w = y_target - L3 * np.sin(phi_target)

    r2 = x_w ** 2 + y_w ** 2

    cos_theta2 = (r2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)

    if abs(cos_theta2) > 1.0:
        return None

    sin_theta2 = np.sqrt(1 - cos_theta2 ** 2)

    if not elbow_up:
        sin_theta2 = -sin_theta2

    theta2 = np.arctan2(sin_theta2, cos_theta2)

    theta1 = np.arctan2(y_w, x_w) - np.arctan2(L2 * sin_theta2, L1 + L2 * cos_theta2)

    theta3 = phi_target - theta1 - theta2

    return theta1, theta2, theta3


def shortest_angle_diff(target, current):
    """Compute minimal signed angular difference (-pi to pi)."""
    return (target - current + np.pi) % (2 * np.pi) - np.pi


def generate_path(pattern="circle", num_points=20):
    """Generate a list of (x, y) points for the arm to follow."""
    global L1, L2, L3
    radius = L1 + L2 + L3
    points = []

    if pattern == "random":
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


def proportional_update_arm(angles):
    global THETA1, THETA2, THETA3
    t1_target, t2_target, t3_target = angles
    THETA1 += Kp * shortest_angle_diff(t1_target, THETA1)
    THETA2 += Kp * shortest_angle_diff(t2_target, THETA2)
    THETA3 += Kp * shortest_angle_diff(t3_target, THETA3)
    update_arm(THETA1, THETA2, THETA3)


def update_arm(t1, t2, t3):
    """Update arm, target dot, and orientation arrow."""
    global arm_line, joints
    x_values, y_values = forward_kinematics(t1, t2, t3)
    arm_line.set_data(x_values, y_values)
    joints.set_data(x_values, y_values)

    draw_arrow(t1, t2, t3)
    fig.canvas.draw_idle()


def draw_arrow(t1, t2, t3):
    global orientation_arrow
    x_values, y_values = forward_kinematics(t1, t2, t3)
    phi = t1 + t2 + t3
    orientation_arrow.remove()
    orientation_arrow = ax.arrow(
        x_values[-1],
        y_values[-1],
        1.0 * np.cos(phi),
        1.0 * np.sin(phi),
        head_width=0.6,
        head_length=0.8,
        fc='black',
        ec='black'
    )
    return orientation_arrow


def path_mode(pattern="circle"):
    """Automatically follow a predefined path."""
    points = generate_path(pattern)
    for px, py in points:
        ax.plot(px, py, 'o', color='red', markersize=4)

    path_idx = [0]
    tolerance = 0.05

    def timer_event(_):
        nonlocal points, path_idx

        point_x, point_y = points[path_idx[0]]

        phi_target = np.arctan2(point_y, point_x)

        angles = inverse_kinematics(point_x, point_y, phi_target)

        if angles is None:
            path_idx[0] = (path_idx[0] + 1) % len(points)
            return

        proportional_update_arm(angles)

        x_values, y_values = forward_kinematics(THETA1, THETA2, THETA3)
        x_end_effector, y_end_effector = x_values[-1], y_values[-1]

        if np.hypot(point_x - x_end_effector, point_y - y_end_effector) < tolerance:
            path_idx[0] = (path_idx[0] + 1) % len(points)

    timer = fig.canvas.new_timer(interval=int(dt * 1000))
    timer.add_callback(timer_event, None)
    timer.start()
    plt.show()


def setup():
    """Set up the simulation environment."""
    global ax, arm_line, joints, THETA1, THETA2, THETA3, orientation_arrow
    ax.set_aspect('equal')
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_title("3DOF Arm Simulation")
    ax.grid(True)

    x, y = forward_kinematics(THETA1, THETA2, THETA3)
    arm_line, = ax.plot(x, y, 'o-', color='black', linewidth=3)
    joints, = ax.plot(x, y, 'o', color='blue', markersize=5)

    orientation_arrow = draw_arrow(THETA1, THETA2, THETA3)


# ------------------- Main -------------------
def main():
    setup()
    path_mode(pattern="random")


if __name__ == "__main__":
    main()
