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

    while dist_formula_2d(pos, trav) < dist_formula_2d(pos, prev):
        prev = trav
        trav = trav.getNext()
    while dist_formula_2d(pos, trav) < look_ahead_distance:
        prev = trav
        trav = trav.getNext()

    lastLooked = trav
    # finding distance between us and where we want to go
    # (pos[0] - trav[0]) + (pos[1]] - trav[1]] + #(pos.getZ() - trav.getZ())
    # inverse (cos u . v) / (mag u) (mag v)

    # making vector 1 2d
    heading = velocity[:2]
    unit_heading = heading / np.linalg.norm(heading)

    unit_y_axis = [1, 0]

    dot_product = np.dot(unit_heading, unit_y_axis)
    angle = np.arccos(dot_product)

    point = trav[:2] - pos[:2]

    # rotation matrix code
    rotX = (point[0] * np.cos(angle)) - (point[1] * np.sin(angle))
    # rotY = (point[0]] * np.sin(angle)) + (point[1]] * np.cos(angle))

    radius = (dist_formula_2d(pos, trav.getKey())) / 2 * rotX

    # TODO: determine number of waypoints to return
    num_points_created = 10
    # CREATES WAYPOINTS

    return lastLooked, [(pos[0] + trav.getKey()[0] - (radius**2 - y**2), pos[1] + y) for y in range(num_points_created)]


main()
