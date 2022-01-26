import math

import cart as cart
import numpy as np

# actually make work at some point
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


def dist_formula_2d(pos, point):
    return math.sqrt((pos[0] - point[0]) ** 2 + (pos[1] - point[1]) ** 2)  # + (pos.getZ() - point.getZ()) ** 2)


def main(pos, look_ahead_distance, velocity):
    # TODO: Last looked is a global variable that is the head of an updated linked list storing the optimal path
    global lastLooked
    prev = lastLooked

    # traversal variable
    trav = lastLooked.get_next()

    # go from the last point we were looking at to the point closest to the payload
    while dist_formula_2d(pos, trav.get_value()) < dist_formula_2d(pos, prev.get_value()):
        prev = trav
        trav = trav.get_next()

    # go from point inside circle (closest to payload) to the edge of the circle
    while dist_formula_2d(pos, trav.get_value()) < look_ahead_distance:
        trav = trav.get_next()

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

    point = np.subtract(trav.get_value()[:2], pos[:2])
    point = np.ndarray.tolist(point)

    # rotation matrix code
    rot_x = (point[0] * np.cos(angle)) - (point[1] * np.sin(angle))
    rot_y = (point[0] * np.sin(angle)) + (point[1] * np.cos(angle))

    zero_avoidance = 1
    print("Position: ", pos)
    print("Target: ", trav.get_value())
    print("Distance: ", point)
    print("Target Rotated: ", [rot_x, rot_y])

    radius = abs((dist_formula_2d([0, 0], [rot_x, rot_y])**2) / (2 * (np.sign(rot_x) * (abs(rot_x) + zero_avoidance))))

    # TODO: determine number of waypoints to return
    num_points_created = 10
    # CREATES WAYPOINTS

    waypoints = [0] * num_points_created

    # Create waypoint in the transformed coordinates. Then, use the inverse of the rotation matrix to rotate it back
    print("rot_x: ", rot_x)
    print("rot_y: ", rot_y)
    print("Radius: ", radius)

    for i in range(num_points_created):
        # Calculating the target point within rotated coordinates
        target_rot_y = rot_y * (i / (num_points_created-1))
        if(rot_x) > 0:
            target_rot_x = radius - math.sqrt((radius ** 2) - (target_rot_y ** 2))
        else:
            target_rot_x = -(radius - math.sqrt((radius ** 2) - (target_rot_y ** 2)))


        # print("X: ", target_rot_x)
        # print("Y: ", target_rot_y)

        # Translating coordinates back to cartesian coordinates
        cartesian_x = (target_rot_x * np.cos(-angle)) - (target_rot_y * np.sin(-angle))
        cartesian_y = (target_rot_x * -np.sin(-angle)) + (target_rot_y * np.cos(-angle))

        # print("Cartesian X: ", cartesian_x)
        # print("Cartesian Y: ", cartesian_y)

        waypoints[i] = [cartesian_x, cartesian_y]

    return waypoints


if __name__ == "__main__":

    for i in [199 - i for i in range(200)]:
        lastLooked = Node([i, i**2], lastLooked)

    pos = [0, 0]
    direction = [0, 1]
    for i in range(10):
    # if True:
        print(f"\nIteration {i+1}")
        waypoints = main(pos, 10, direction)
        print(waypoints)
        pos = waypoints[6]
        direction = np.subtract(waypoints[6], waypoints[5])


