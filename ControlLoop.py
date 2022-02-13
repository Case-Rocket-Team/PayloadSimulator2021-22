
# Kyler Rosen
# v0.1 Finished Path Algorithm

from math import sqrt, pi, floor

# these are just for plotting
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import numpy as np


class Node:
    # Function to initialise the node object
    def __init__(self, data, next_node=None):
        self.data = data  # Assign data
        self.next = next_node  # Initialize next as null

    def get_next(self):
        return self.next

    def get_value(self):
        return self.data


def dead_reckon(gps, imu, pressure, pos_prev, vel_prev, accel_prev):
    """integrate position from of the craft from sensor data

    Args:
        gps (Vec2): the generated noisy gps data
        imu (Vec3): the generated noisy imu accelerations
        pressure (float): the generated noisy altitude data
        pos_prev (Vec3[]): a history of the previous positions
        vel_prev (Vec3[]): a history of the previous velocities
        accel_prev (Vec3[]): a history of the previous accelerations

    Returns:
        pos_list: list of positions with new position appended
        vel_list: list of velocities with new velocity appended
        accel_list: list of accelerations with new acceleration appended
    """
    return pos_list, vel_list, accel_list


def dist_formula_2d(pos, point):
    return sqrt((pos[0] - point[0]) ** 2 + (pos[1] - point[1]) ** 2)  # + (pos.getZ() - point.getZ()) ** 2)


def pure_pursuit(pos, vel, accel, path):
    """Find the appropriate direction to go to the next waypoint, using pure pursuit

    Args:
        pos (Vec3): the position of the craft
        vel (Vec): the current velocity of the craft
        accel (Vec): the current acceleration of the craft
        path (Vec3[]): the set of waypoints

    Returns:
        accel_hat: a unit vector of the direction to go
    """
    return accel_hat


# takes the cross product of vectors a, b, and c
def cross_product(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[1] - a[1])


# finds how much higher / lower the payload would be expected to land at the given specications
def error_margin_expected(expected_height, guess_r, loop_num, dz_dr):
    return expected_height - 2 * pi * guess_r * loop_num * dz_dr


# normalizes vector v
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v

    return v / norm


# generates a helical pattern downwards
def generate_points(loop_num, step_per_circle, x_0, y_0, r, starting_height, path, starting_point, dz_dr, clockwise):
    # rewrite of law of consines, see attatched
    theta_offset = np.arccos(1 - dist_formula_2d([x_0 + r, y_0], starting_point) ** 2 / (2 * r ** 2))

    if clockwise:
        multiplier = -1

    else:
        multiplier = 1

    for i in range(0, floor(step_per_circle * loop_num)):
        theta = multiplier * (2 * np.pi * loop_num * i / floor(step_per_circle * loop_num)) - theta_offset
        x = x_0 + r * np.cos(theta)
        y = y_0 + r * np.sin(theta)
        z = starting_height - (((theta + theta_offset) * multiplier) * r) * dz_dr
        path.append([x, y, z])


def gen_path(pos, vel, target_loc, turn_radius=147, num_waypoints=1000):
    """Generate a set of waypoints for the craft to follow

    Args:
        pos (Vec3): the current position.
        vel (Vec3): the craft's velocity.
        turn_radius (float): the turning radius of the craft .
        target_loc (Vec3): the target location to aim towards.
        num_waypoints (int): the number of waypoints to generate on the path.

    Returns:
        path: the set of waypoints generated
    """
    path = []
    dz_dr = 1/2.7
    step_per_circle = 50
    vel = normalize(vel)

    print(f"\nTurning Towards Target: ")

    # find whether the point it is targetting is to the left or right
    turn_right = True
    if cross_product([0, 0], vel, target_loc) > 0:
        turn_right = False
    print(f"turn_right: {turn_right}:")

    # while tangent of minimum radius circle does not point towards target:
    # move 1 step farther on circle
    # reminder that in a circle, dy/dx = -x/y
    dx = -vel[0]
    dy = vel[1]

    # finding the center of the circle based off of the slope at its current position
    # off condition that dx = 0 which would lead to an infinite slope
    if dx == 0:
        y_0 = pos[1]
        x_0 = pos[0] + turn_radius ** 2

    # dy/dx = -x/y so x = -dy/dx * y
    # +-r = sqrt((x-x_0)^2 + (y-y_0)y^2), but x,y = 0,0 so +-r = sqrt((x_0)^2 + (y_0)^2)
    # rewrite +-r = sqrt((x_0)^2 + (y_0)^2) = sqrt((-dy/dx * y_0)^2 + (y_0)^2) = y_0 * sqrt((dy/dx)^2 + 1)

    # condition that r is negative, ei a vector pointing left from the center to the point
    elif turn_right:
        tangent_slope = dy/dx
        y_0 = pos[1] - turn_radius / sqrt(1 + tangent_slope**2)
        x_0 = pos[0] + turn_radius**2 - y_0**2

    # condition that the radius is the
    else:
        tangent_slope = dy / dx
        y_0 = pos[1] + turn_radius / sqrt(1 + tangent_slope ** 2)
        x_0 = pos[0] + turn_radius ** 2 - y_0 ** 2

    print(f"x_0: {x_0}")
    print(f"y_0: {y_0}")

    # see this answer for finding the tangent line off a circle that intersects a point
    # https://www.quora.com/What-is-the-point-of-intersection-of-the-tangents-drawn-at-the-points-where-the-given-line-intersects-the-given-circle

    radius_to_target = np.subtract(target_loc[:2], [x_0, y_0])
    p = dist_formula_2d([0, 0], radius_to_target)

    print(f"turn_radius")
    print(f"radius_to_target: {radius_to_target}")
    print(f"p = {p}")
    c = turn_radius**2/p
    print(f"c = {c}")
    h = - sqrt(turn_radius**2 - c**2)

    print(f"vel: {vel}")
    norm_radius_to_target = normalize(radius_to_target)
    c_vector = [c * norm_radius_to_target[0], c * norm_radius_to_target[1]]
    h_vector = [h * norm_radius_to_target[1], h * -norm_radius_to_target[0]]
    delta = np.add(c_vector, h_vector)
    delta = np.ndarray.tolist(delta)

    print(f"[x_0, y_0]: {[x_0, y_0]}")
    print(f"delta: {delta}")

    tangent_point = np.add([x_0, y_0], delta)
    print(dist_formula_2d([x_0, y_0], tangent_point))

    rotated_velocity = [None, None]
    rotated_velocity[1], rotated_velocity[0] = vel[0], vel[1]

    if cross_product([0, 0], rotated_velocity, tangent_point) < 0:
        print(f"h: {h}")
        correction_vector = [None, None]
        correction_vector[0], correction_vector[1] = -2 * h_vector[0], -2 * h_vector[1]
        print(f"c: {c}")
        print(f"h: {h}")
        tangent_point = np.add(tangent_point, correction_vector)

    print(dist_formula_2d([x_0, y_0], tangent_point))
    print(f"This thing: {1 - (dist_formula_2d(pos, tangent_point)**2) / (2 * turn_radius**2)}")
    theta = np.arccos(1 - dist_formula_2d(pos, tangent_point)**2 / (2 * turn_radius**2))
    arc_length = theta * turn_radius
    loops_necessary = arc_length / (2 * pi * turn_radius)

    generate_points(loops_necessary, step_per_circle, x_0, y_0, turn_radius, pos[2], path, pos, dz_dr, turn_right)

    # final vector saved in tangent_point
    straight_path_direction = np.subtract(target_loc[:2], tangent_point)
    norm_straight_path_direction = normalize(straight_path_direction)

    # finds how much height is expected to be lsot
    # see this link for arc length calculations:
    # https://math.stackexchange.com/questions/830413/calculating-the-arc-length-of-a-circle-segment

    # generate path between the tangent point and the target point
    step = 10
    height = pos[2] - arc_length * dz_dr

    for i in range(0, floor(dist_formula_2d([0, 0], straight_path_direction)), step):
        point = tangent_point[:2] + i * norm_straight_path_direction
        point = np.ndarray.tolist(point)
        point = [point[0], point[1], height - i * dz_dr]
        path.append(point)

    zach_leclaire = "🤰"

    length = arc_length + dist_formula_2d(tangent_point, target_loc)
    expected_height = pos[2] - length * dz_dr
    guess_r = 1.5 * turn_radius
    loops_necessary = round(expected_height / (2 * pi * guess_r * dz_dr))

    error_margin = error_margin_expected(expected_height, guess_r, loops_necessary, dz_dr)
    h = 1

    acceptable_error_margin = 10
    while abs(error_margin) > acceptable_error_margin:
        error_margin_plus_h = error_margin_expected(expected_height, guess_r + h, loops_necessary, dz_dr)
        error_margin = error_margin_expected(expected_height, guess_r, loops_necessary, dz_dr)
        error_derivative = (error_margin_plus_h - error_margin) / h
        guess_r = guess_r - error_margin / error_derivative

    x_1 = target_loc[0] + guess_r * norm_straight_path_direction[1]
    y_1 = target_loc[1] + -1 * guess_r * norm_straight_path_direction[0]

    print(dist_formula_2d([x_1, y_1], target_loc))

    generate_points(loops_necessary, step_per_circle, x_1, y_1, guess_r, expected_height, path, target_loc, dz_dr, True)

    return path


def plotting():
    path = gen_path([0, 0, 2000], [50, 0, 0], [0, 10, 0])
    print(path)

    path_x = [item[0] for item in path[:-1]]
    path_y = [item[1] for item in path[:-1]]
    path_z = [item[2] for item in path[:-1]]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(path_x, path_y, path_z)

    axes = plt.gca()

    axes.set_xlabel('X')
    axes.set_ylabel('Y')
    axes.set_zlabel('Z')

    min_coord = -1000
    max_coord = 3000
    step = 500

    x_ticks = np.arange(min_coord, max_coord, step)
    plt.xticks(x_ticks)

    y_ticks = np.arange(min_coord, max_coord, step)
    plt.yticks(y_ticks)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plotting()
