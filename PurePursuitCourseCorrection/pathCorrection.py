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

    # go from point inside circle (closest to payload) to the farthest point within the lookahead distace
    while dist_formula_2d(pos, trav.get_value()) < look_ahead_distance:
        trav = trav.get_next()

    lastLooked = trav
    # finding distance between us and where we want to go

    # making heading 2d, normalizing
    heading = velocity[:2]
    unit_heading = heading / np.linalg.norm(heading)

    # creating y-axis basis vector
    unit_y_axis = [0, 1]

    # using that cos(theta) = (u . v) / (||u|| ||v||)
    # note ||u|| = ||v|| = 1
    dot_product = np.dot(unit_heading, unit_y_axis)
    angle = np.arccos(dot_product)

    # modifying our location to (0,0)
    point = np.subtract(trav.get_value()[:2], pos[:2])
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
    # if there is an error, it is here probably. Recheck photo
    bank_angle = math.asin((velocity[0] ^ 2)/(9.81 * radius))

    # TODO: determine number of waypoints to return
    num_points_created = 10

    # CREATES WAYPOINTS
    waypoints = [0] * num_points_created

    # Create waypoint in the transformed coordinates. Then, use the inverse of the rotation matrix to rotate it back

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

        coords = np.add([cartesian_x, cartesian_y], pos[:2])
        coords = np.ndarray.tolist(coords)

        waypoints[i] = coords

    return waypoints, bank_angle


if __name__ == "__main__":
    pointNum = 20000
    for i in [pointNum - 1 - i for i in range(pointNum)]:
        lastLooked = Node([i, math.sqrt(i)], lastLooked)

    # iterations and lookahead dist will need to be adjusted based on function and simulations
    pos = [10, 2]
    direction = [12, 5]
    for i in range(100):
    # if True:
        print(f"\nIteration {i+1}")
        waypoints = main(pos, 50, direction)
        print(waypoints)
        pos = waypoints[9]
        direction = np.subtract(waypoints[9], waypoints[8])


