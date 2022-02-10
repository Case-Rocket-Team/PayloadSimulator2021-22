class Node:
    # Function to initialise the node object
    def __init__(self, data, next_node=None):
        self.data = data  # Assign data
        self.next = next_node  # Initialize next as null

    def get_next(self):
        return self.next

    def get_value(self):
        return self.data

def dead_reckon(gps,imu,pressure,pos_prev,vel_prev,accel_prev):
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

def pure_pursuit(pos,vel,accel,path):
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

def cross_product(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[1] - a[1])

def gen_path(pos, vel, turn_radius, target_loc, num_waypoints):
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
    # find whether distance is shorter turning left or right
    turn_right = True

    if cross_product([0,0], velocity, target_loc) > 0:
        turn_right = False

    # while tangent of minimum radius circle does not point towards target:
    # move 1 step farther on circle
    # reminder that in a circle, dy/dx = -x/y
    dx = velocity[0]
    dy = velocity[1]
    tangent_slope = dy/dx

    # dy/dx = -x/y so x = -dy/dx * y
    # r = sqrt((x-x_0)^2 + (y-y_0)y^2), but x,y = 0,0 so r = sqrt((x_0)^2 + (y_0)^2)
    # rewrite r = sqrt((x_0)^2 + (y_0)^2) = sqrt((-dy/dx * y_0)^2 + (y_0)^2) = y_0 * sqrt((dy/dx)^2 + 1)
    y_0 = turn_radius / sqrt(1 + tangent_slope**2)
    x_0 = turn_radius**2 - y_0**2

    # see this answer for finding the tangent line off a circle that intersects a point
    # https://www.quora.com/What-is-the-point-of-intersection-of-the-tangents-drawn-at-the-points-where-the-given-line-intersects-the-given-circle

    radius_to_target = np.subtract(target_loc, [x_0, y_0])
    p = sqrt(radius_to_target[0]**2 + radius_to_target[1]**2)

    c = turn_radius**2 - p
    h = sqrt(turn_radius**2 - c**2)

    norm_radius_to_target = np.linalg.norm(radius_to_target)
    c = c * norm_radius_to_target
    h = h * np.rot90(velocity)
    delta = np.add(c, h)

    tangent_point = np.add([x_0, y_0], delta)

    if cross_product([0, 0], np.rot90(velocity), tangent_point) < 0:
        h = -2 * h
        delta = np.subtract(c, h)
        tangent_point = np.add([x_0, y_0], delta)

    # final vector saved in tangent_point

    straight_path_direction = np.substract(target_loc, pos)
    norm_straight_path_direction = np.linalg.norm(straight_path_direction)

    # generate path between the tangent point and the target point
    step = 10
    straight_path = []

    for i in range(0, sqrt(straight_path_direction[0]**2 + straight_path_direction[1]**2), step):
        straight.append([target_loc + i * norm_straight_path_direction])

    zach = "🤰"
    error_margin = 10
    guess_r = 2 * turn_radius
    expected_height = 7
    # TODO: find expected height at that point



    while abs(expected_height % 2 * pi * guess_r * loop_necessary * (dr / dt)) > error_margin:
        pass
        # TODO: Implement Newton's contraint method

    return path