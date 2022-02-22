
# Kyler Rosen
# v0.1 Finished Path Algorithm

from math import sqrt, pi, floor, asin

# these are just for plotting
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import numpy as np

lastLooked = None

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

    Paul: position estimate from acceleration
        calculate reimann sum on accel twice

    Returns:
        pos_list: list of positions with new position appended
        vel_list: list of velocities with new velocity appended
        accel_list: list of accelerations with new acceleration appended
    """
    return pos_list, vel_list, accel_list


def dist_formula_2d(pos, point):
    return sqrt((pos[0] - point[0]) ** 2 + (pos[1] - point[1]) ** 2)  # + (pos.getZ() - point.getZ()) ** 2)


def pure_pursuit(pos, look_ahead_distance, velocity, path):
    # TODO: Last looked is a global variable that is the head of an updated linked list storing the optimal path
    global lastLooked

    # go from the last point we were looking at to the point closest to the payload
    while dist_formula_2d(pos, path[lastLooked + 1]) < dist_formula_2d(pos, path[lastLooked]):
        lastLooked += 1

    # go from point inside circle (closest to payload) to the farthest point within the lookahead distace
    while dist_formula_2d(pos, path[lastLooked + 1]) < look_ahead_distance:
        lastLooked += 1

    lastLooked += 1
    # finding distance between us and where we want to go

    # making heading 2d, normalizing
    heading = velocity[:2]
    unit_heading = normalize(heading)

    # creating y-axis basis vector
    unit_y_axis = [0, 1]

    # using that cos(theta) = (u . v) / (||u|| ||v||)
    # note ||u|| = ||v|| = 1
    dot_product = np.dot(unit_heading, unit_y_axis)
    angle = np.arccos(dot_product)

    # modifying our location to (0,0)
    point = np.subtract(path[lastLooked][:2], pos[:2])
    point = np.ndarray.tolist(point)

    # rotating basis vectors so that payload faces in [0,1] direction
    rot_x = (point[0] * np.cos(angle)) - (point[1] * np.sin(angle))
    rot_y = (point[0] * np.sin(angle)) + (point[1] * np.cos(angle))

    # if x is too small, radius is too large. So, increase the size of x slightly
    # adjust this as needed
    zero_avoidance = 0.00001
    if abs(rot_x) < zero_avoidance:
        rot_x = np.sign(rot_x) * zero_avoidance

    # Using the inverse for the equation for curvature of a circle
    radius = abs((dist_formula_2d([0, 0], [rot_x, rot_y])**2) / (2 * rot_x))

    # minimum turn radius, check after testing
    min_turn_rad = 69.4

    # stops running if we are so far off the path that we need too tight of a radius. They need to rerun algorithm
    if radius < min_turn_rad:
        return [], None

    # determines bank angle off of
    # TODO: CITE TEXTBOOK
    #if there is an error, it is here probably. Recheck photo
    bank_angle = asin((velocity[0] ^ 2)/(9.81 * radius))

    # TODO: determine number of waypoints to return
    num_points_created = 10
    # CREATES WAYPOINTS

    waypoints = [0] * num_points_created

    # Create waypoint in the transformed coordinates. Then, use the inverse of the rotation matrix to rotate it back

    for i in range(num_points_created):
        # Calculating the target point within rotated coordinates
        target_rot_y = rot_y * (i / (num_points_created-1))
        if(rot_x) > 0:
            target_rot_x = radius - sqrt((radius ** 2) - (target_rot_y ** 2))
        else:
            target_rot_x = -(radius - sqrt((radius ** 2) - (target_rot_y ** 2)))


        # print("X: ", target_rot_x)
        # print("Y: ", target_rot_y)

        # Translating coordinates back to cartesian coordinates
        cartesian_x = (target_rot_x * np.cos(-angle)) - (target_rot_y * np.sin(-angle))
        cartesian_y = (target_rot_x * -np.sin(-angle)) + (target_rot_y * np.cos(-angle))

        # print("Cartesian X: ", cartesian_x)
        # print("Cartesian Y: ", cartesian_y)

        coords = np.add([cartesian_x, cartesian_y], pos[:2])
        coords = np.ndarray.tolist(coords)

        waypoints[i] = coords

    return waypoints, bank_angle


# takes the cross product of vectors a, b, and c
def cross_product(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1])) - ((b[1] - a[1]) * (c[0] - a[0]))


# finds how much higher / lower the payload would be expected to land at the given specifications
def error_margin_expected(expected_height, guess_r, loop_num, dz_dr):
    return expected_height - 2 * pi * guess_r * loop_num * dz_dr


# normalizes vector v
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v

    return v / norm


# generates a helical pattern downwards
def generate_helix(loop_num, step_per_circle, x_0, y_0, r, starting_height, path, starting_point, dz_dr, clockwise, target_point = None):
    # rewrite of law of cosines, see attatched
    # https://math.stackexchange.com/questions/830413/calculating-the-arc-length-of-a-circle-segment


    theta_offset = np.arccos(1 - dist_formula_2d([x_0 + r, y_0], starting_point) ** 2 / (2 * r ** 2))

    if starting_point[1] > y_0:
        theta_offset = 2 * pi - theta_offset

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

    # adjusts if arccos is off because of range

    error_margin = 10

    if target_point is not None:
        if dist_formula_2d([x, y, z], target_point) < error_margin:
            return

        theta = loop_num * 2 * pi
        theta = (2 * pi - theta) - theta
        loop_num = theta / (2 * pi)

        generate_helix(loop_num, step_per_circle, x_0, y_0, r, z, path, [x, y], dz_dr, clockwise)


def generate_straight_path(pos, arc_length, dz_dr, straight_path_direction, tangent_point, norm_straight_path_direction, path):
    print("")
    print("Generating a straight path: ")
    # generate path between the tangent point and the target point
    step = 10
    height = pos[2] - arc_length * dz_dr
    print(f"Height: {height}")

    for i in range(0, floor(dist_formula_2d([0, 0], straight_path_direction)), step):
        point = tangent_point[:2] + i * norm_straight_path_direction

        point = np.ndarray.tolist(point)
        point = [point[0], point[1], height - i * dz_dr]
        path.append(point)


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
    print(f"vel: {vel}")

    print(f"\n\nTurning Towards Target: ")

    # find whether the point it is targeting is to the left or right
    turn_right = True
    if cross_product([0, 0], vel, target_loc) > 0:
        turn_right = False

    print(f"turn_right: {turn_right}:")

    # rotates heading based off of turn direction to generate center point
    radius = [None, None]

    if turn_right:
        radius_direction = vel[1], -vel[0]

    else:
        radius_direction = -vel[1], vel[0]

    x_0 = pos[0] + turn_radius * radius_direction[0]
    y_0 = pos[1] + turn_radius * radius_direction[1]

    print(f"x_0: {x_0}")
    print(f"y_0: {y_0}")

    if dist_formula_2d([x_0, y_0], target_loc) < turn_radius:
        loops_necessary = round(pos[2] / (2 * pi * turn_radius * dz_dr))
        generate_helix(loops_necessary, step_per_circle, x_0, y_0, turn_radius, pos[2], path, pos, dz_dr, turn_right)

        return path

    # see this answer for finding the tangent line off a circle that intersects a point using similar triangles
    # note that c/r = r/p
    # https://www.quora.com/What-is-the-point-of-intersection-of-the-tangents-drawn-at-the-points-where-the-given-line-intersects-the-given-circle

    # finds the vector pointing from the center of the circle to the target and its magnitude
    radius_to_target = np.subtract(target_loc[:2], [x_0, y_0])
    norm_radius_to_target = normalize(radius_to_target)
    p = dist_formula_2d([0, 0], radius_to_target)

    print("")
    print(f"radius_to_target: {radius_to_target}")
    print(f"p = {p}")

    # finds the projection of the vector from the center of the circle to the tangent point onto radius_to_target
    c = turn_radius**2/p
    print(f"c = {c}")

    # finds the length from the c vector to the end of the radius_to_target
    h = - sqrt(turn_radius**2 - c**2)
    print(f"h = {h}")

    # creates the c vector colinear and perpendicular to radius_to_target respectively
    c_vector = [c * norm_radius_to_target[0], c * norm_radius_to_target[1]]
    h_vector = [h * norm_radius_to_target[1], h * -norm_radius_to_target[0]]
    print(f"c_vector = {c_vector}")
    print(f"h_vector = {h_vector}")

    # finds the radius vector from the center of the circle to the tangent point
    delta = np.add(c_vector, h_vector)
    delta = np.ndarray.tolist(delta)
    print(f"delta: {delta}")
    print("")

    # Finds the point where the tangent line on the circle intersects the target point
    tangent_point = np.add([x_0, y_0], delta)
    print(f"Distance from circle center to tangent point: {dist_formula_2d([x_0, y_0], tangent_point)}")

    # rotates the heading so it points towards / away from the center of the circle
    rotated_velocity = [None, None]
    rotated_velocity[1], rotated_velocity[0] = vel[0], vel[1]

    # checks if tangent point should be on on the other side of the circle and flips the direction of h
    if cross_product([0, 0], rotated_velocity, tangent_point) < 0:
        correction_vector = [None, None]
        correction_vector[0], correction_vector[1] = -2 * h_vector[0], -2 * h_vector[1]
        tangent_point = np.add(tangent_point, correction_vector)

    # finds the arc length of curve found and determines what fraction of a circle it is
    print(f"cos(theta): {1 - (dist_formula_2d(pos, tangent_point)**2) / (2 * turn_radius**2)}")
    theta = np.arccos(1 - dist_formula_2d(pos, tangent_point)**2 / (2 * turn_radius**2))

    arc_length = theta * turn_radius
    loop_fraction_necessary = arc_length / (2 * pi * turn_radius)
    print(f"arc_length: {arc_length}")

    generate_helix(loop_fraction_necessary, step_per_circle, x_0, y_0, turn_radius, pos[2], path, pos, dz_dr, turn_right, tangent_point)

    # finds the vector from the tangent point to the target point and normalizes the direction
    straight_path_direction = np.subtract(target_loc[:2], tangent_point)
    norm_straight_path_direction = normalize(straight_path_direction)

    generate_straight_path(pos, arc_length, dz_dr, straight_path_direction, tangent_point, norm_straight_path_direction, path)

    zach_leclaire = "🤰"

    # finds how much height is expected to be lost
    # see this link for arc length calculations:
    # https://math.stackexchange.com/questions/830413/calculating-the-arc-length-of-a-circle-segment
    length = arc_length + dist_formula_2d(tangent_point, target_loc)
    expected_height = pos[2] - length * dz_dr
    
    if expected_height < 0:
        return path

    # calculates how many loops until the payload hits the ground
    # creates an initial guess
    guess_r = 1.5 * turn_radius
    loops_necessary = floor(expected_height / (2 * pi * guess_r * dz_dr))

    # calculates how far the payload would stop above/below the ground
    error_margin = error_margin_expected(expected_height, guess_r, loops_necessary, dz_dr)
    h = 1

    # Secant method to optimize guess_r
    acceptable_error_margin = 10
    while abs(error_margin) > acceptable_error_margin:
        error_margin_plus_h = error_margin_expected(expected_height, guess_r + h, loops_necessary, dz_dr)
        error_margin = error_margin_expected(expected_height, guess_r, loops_necessary, dz_dr)
        error_derivative = (error_margin_plus_h - error_margin) / h
        guess_r = guess_r - error_margin / error_derivative

    # finds the center of the circle with radius guess_r
    x_1 = target_loc[0] + guess_r * norm_straight_path_direction[1]
    y_1 = target_loc[1] + -1 * guess_r * norm_straight_path_direction[0]

    print(f"distance from center of circle to target: {dist_formula_2d([x_1, y_1], target_loc)}")

    generate_helix(loops_necessary, step_per_circle, x_1, y_1, guess_r, expected_height, path, target_loc, dz_dr, True)

    return path


def plotting():
    path = gen_path([0, 0, 5000], [1, -1, 0], [-500, 500, 0])
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

    min_coord = -3000
    max_coord = 2000
    step = (max_coord - min_coord) // 10

    x_ticks = np.arange(min_coord, max_coord, step)
    plt.xticks(x_ticks)

    y_ticks = np.arange(min_coord, max_coord, step)
    plt.yticks(y_ticks)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plotting()
