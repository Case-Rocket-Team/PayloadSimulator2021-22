import math
import numpy as np

# actually make work at some point
lastLooked = None


def dist_formula_2d(pos, point):
    return math.sqrt((pos[0] - point[0]) ** 2) + ((pos[1] - point[1]) ** 2)  # + (pos.getZ() - point.getZ()) ** 2)


def main(pos, look_ahead_distance, velocity):
    # TODO: Last looked is a global variable that is the head of an updated linked list storing the optimal path
    global lastLooked
    prev = lastLooked

    # traversal variable
    trav = lastLooked.getNext()

    # go from the last point we were looking at to the point closest to the payload
    while dist_formula_2d(pos, trav) < dist_formula_2d(pos, prev):
        prev = trav
        trav = trav.getNext()

    # go from point inside circle (closest to payload) to the edge of the circle
    while dist_formula_2d(pos, trav) < look_ahead_distance:
        trav = trav.getNext()

    lastLooked = trav
    # finding distance between us and where we want to go
    # (pos[0] - trav[0]) + (pos[1]] - trav[1]] + #(pos.getZ() - trav.getZ())
    # inverse (cos u . v) / (mag u) (mag v)

    # making heading 2d
    heading = velocity[:2]
    unit_heading = heading / np.linalg.norm(heading)

    unit_y_axis = [0, 1]

    dot_product = np.dot(unit_heading, unit_y_axis)
    angle = np.arccos(dot_product)

    point = trav[:2] - pos[:2]

    # rotation matrix code
    rot_x = (point[0] * np.cos(angle)) - (point[1] * np.sin(angle))
    # rot_y = (point[0] * np.sin(angle)) + (point[1] * np.cos(angle))

    radius = (dist_formula_2d(pos, trav.getKey())**2) / (2 * rot_x)

    # TODO: determine number of waypoints to return
    num_points_created = 10
    # CREATES WAYPOINTS

    waypoints = [] * num_points_created

    # Create waypoint in the transformed coordinates. Then, use the inverse of the rotation matrix to rotate it back
    for y in range(num_points_created):
        target_rot_x = rot_x + (radius ** 2 - y ** 2)
        target_rot_y = y

        cartesian_x = (target_rot_x * np.cos(-angle)) - (target_rot_y * np.sin(-angle))
        cartesian_y = (target_rot_x * np.sin(-angle)) + (target_rot_y * np.cos(-angle))
        waypoints[y] = [cartesian_x, cartesian_y]




    return lastLooked, waypoints
